"""One goal evaluator for roam (operator 2026-11-24: "one place decides").

Every source - scan-walk corridors, held heading, frontiers/patrol, the course grid's "come back" memory, scene
hints - only PROPOSES candidates. This module merges near-duplicates (keeping every contributing reason), scores
them with configurable weights, and returns one decision with the score breakdown of the chosen goal and its
alternatives. Roam stays the only thing that moves the robot; Reselection decides when a better goal may replace
the one being walked (hysteresis, so small score changes never cancel productive movement). Pure: unit-tested.
"""

import math
from dataclasses import dataclass, field

# Starting weights; tune on saved maps (--weights "info=1,camera=0.8,..."). Gains are capped at GAIN_CAP_M2.
DEFAULT_WEIGHTS = {'info': 1.0, 'camera': 0.8, 'grid': 1.5, 'continue': 0.5, 'agree': 0.3, 'scene': 2.0,
                   'travel': 0.4, 'turn': 0.3, 'terrain': 1.0, 'revisit': 1.0, 'fail': 2.0, 'phase': 2.0}
# phase (operator 2026-11-25): 1 for goals that serve the current phase - search (unfinished cells, unread rooms)
# while any are left; move on (frontier, corridors, patrol) once every reachable cell is inspected and room read.
GAINS = ('info', 'camera', 'grid', 'continue', 'scene', 'phase')
COSTS = ('travel', 'turn', 'terrain', 'revisit', 'fail')
GAIN_CAP_M2 = 4.0
MERGE_M = 0.5
MERGE_YAW_DEG = 45.0
MIN_SCORE = 0.2                      # below this nothing is worth walking to: the stuck ladder takes over


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    source: str                                  # straight_on, corridor, frontier, patrol, grid_return, scene, ...
    reason: str = ''
    terms: dict = field(default_factory=dict)    # term -> raw value (before weights)
    goal: dict = field(default_factory=dict)     # the roam goal dict to send if chosen
    sources: list = field(default_factory=list)
    reasons: list = field(default_factory=list)

    def __post_init__(self):
        self.sources = self.sources or [self.source]
        self.reasons = self.reasons or ([self.reason] if self.reason else [])


@dataclass
class Decision:
    chosen: object                               # Candidate or None
    score: float
    breakdown: dict
    ranked: list                                 # [(score, Candidate, breakdown)] best first

    def summary(self, alternatives=4):
        def one(score, c, b):
            return {'source': '+'.join(c.sources), 'x': round(c.x, 2), 'y': round(c.y, 2),
                    'score': round(score, 2), 'terms': {k: round(v, 2) for k, v in b.items() if v}}
        return {'chosen': one(self.score, self.chosen, self.breakdown) if self.chosen else None,
                'alternatives': [one(*r) for r in self.ranked[1:1 + alternatives]]}


def parse_weights(text):
    """'info=1,camera=0.8' -> full weight dict (unknown names refused, values finite and >= 0)."""
    weights = dict(DEFAULT_WEIGHTS)
    for part in filter(None, (p.strip() for p in (text or '').split(','))):
        name, _, value = part.partition('=')
        name = name.strip()
        if name not in DEFAULT_WEIGHTS:
            raise ValueError(f'unknown weight {name!r} (weights: {", ".join(DEFAULT_WEIGHTS)})')
        v = float(value)
        if not math.isfinite(v) or v < 0:
            raise ValueError(f'weight {name} must be finite and >= 0')
        weights[name] = v
    return weights


def _yaw_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def merge(candidates, dist_m=MERGE_M, yaw_deg=MERGE_YAW_DEG):
    """Near-duplicate proposals become one candidate: every source and reason kept, each term the MAX of the
    contributors (no double counting); 'agree' counts the extra sources. The first proposal keeps its goal."""
    out = []
    for c in candidates:
        for m in out:
            if math.hypot(c.x - m.x, c.y - m.y) <= dist_m and _yaw_diff(c.yaw, m.yaw) <= math.radians(yaw_deg):
                for s in c.sources:
                    if s not in m.sources:
                        m.sources.append(s)
                m.reasons += [r for r in c.reasons if r not in m.reasons]
                for k, v in c.terms.items():
                    if k in COSTS:
                        m.terms[k] = min(m.terms.get(k, v), v)
                    else:
                        m.terms[k] = max(m.terms.get(k, v), v)
                break
        else:
            out.append(Candidate(c.x, c.y, c.yaw, c.source, terms=dict(c.terms), goal=dict(c.goal),
                                 sources=list(c.sources), reasons=list(c.reasons)))
    for m in out:
        m.terms['agree'] = float(len(m.sources) - 1)
    return out


def score(c, weights):
    b = {}
    for k in GAINS + ('agree',):
        v = c.terms.get(k, 0.0)
        if k in ('info', 'camera'):
            v = min(v, GAIN_CAP_M2)
        b[k] = weights.get(k, 0.0) * v
    for k in COSTS:
        b[k] = -weights.get(k, 0.0) * c.terms.get(k, 0.0)
    return sum(b.values()), b


def evaluate(candidates, weights=None, min_score=MIN_SCORE):
    weights = weights or DEFAULT_WEIGHTS
    ranked = []
    for c in merge(candidates):
        total, breakdown = score(c, weights)
        ranked.append((total, c, breakdown))
    ranked.sort(key=lambda r: -r[0])
    if not ranked or ranked[0][0] < min_score:
        return Decision(None, ranked[0][0] if ranked else 0.0, ranked[0][2] if ranked else {}, ranked)
    best = ranked[0]
    return Decision(best[1], best[0], best[2], ranked)


@dataclass
class Reselection:
    """When may a better goal replace the one being walked? Only when the current one became invalid, or a new one
    is clearly better and the current goal had a fair go (operator 2026-11-24: no goal thrashing)."""
    ratio: float = 1.3
    margin: float = 1.0
    commit_s: float = 6.0
    gap_s: float = 10.0
    near_goal_m: float = 1.0
    last_switch: float = -math.inf

    def should_switch(self, now, current_score, best_score, started_at, to_goal_m, invalid=False):
        if invalid:
            return True, 'current goal no longer valid'
        if now - started_at < self.commit_s:
            return False, 'committed'
        if now - self.last_switch < self.gap_s:
            return False, 'switched recently'
        if to_goal_m <= self.near_goal_m:
            return False, 'nearly there'
        if best_score >= current_score * self.ratio + self.margin:
            return True, f'better goal ({best_score:.1f} vs {current_score:.1f})'
        return False, 'not clearly better'

    def switched(self, now):
        self.last_switch = now
