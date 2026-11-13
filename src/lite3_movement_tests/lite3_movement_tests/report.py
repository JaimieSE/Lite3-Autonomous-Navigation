"""JSON (complete record incl. telemetry traces) and Markdown (human summary) reports."""

import json
from pathlib import Path


def _fmt(value):
    return json.dumps(value, sort_keys=True) if isinstance(value, (dict, list)) else str(value)


def markdown(report):
    lines = [
        f'# Movement test report: {report["sequence"]}',
        '',
        f'- **Verdict: {report["verdict"]}**',
        f'- Evidence class: **{report["evidence_class"]}**',
        f'- Mode: {report["mode"]}; started {report["started_utc"]}; duration {report["duration_s"]} s',
        f'- Sequence sha256: `{report["sequence_sha256"]}`',
        f'- Authorization: {report["authorization"]["session"] if report["authorization"] else "none"}',
        f'- Commands sent: {report["commands_sent"]}; posture commands latched off: '
        f'{report["posture_commands_latched_off"]}; state uncertain: {report["state_uncertain"]}',
        '',
        '| Step | Type | Status | Duration s | Summary |',
        '| --- | --- | --- | --- | --- |',
    ]
    for s in report['steps']:
        lines.append(f'| {s["id"]} | {s["type"]} | {s["status"]} | {s.get("duration_s") or ""} | '
                     f'{s["summary"].replace("|", "/")} |')
    failures = [s for s in report['steps'] if s['status'] in ('FAILED', 'ABORTED')]
    lines += ['', '## Failures and remaining issues', '']
    if not failures and report['verdict'] != 'INTERVENTION_REQUIRED':
        lines.append('None in this run.')
    for s in failures:
        m = s.get('measurements', {})
        lines.append(f'- **{s["id"]}** {s["status"]}: {s["summary"]}')
        for key in ('diagnosis', 'measured', 'expect', 'joint_activity_rad', 'peak_planar_speed_m_s',
                    'system_domain_observed', 'problems', 'reasons', 'delivery'):
            if key in m:
                lines.append(f'  - {key}: `{_fmt(m[key])}`')
    if report['verdict'] == 'INTERVENTION_REQUIRED':
        lines.append('- **Operator intervention required**: robot state was not verified safe after a '
                     'command. No further posture command was sent. Check the robot physically, use '
                     'the manufacturer stop/controls if needed, and review telemetry below.')
    if report['safe_stop']:
        lines.append(f'- Safe stop after failure: rest verified = {report["safe_stop"]["rest_verified"]}')
    final = report['final_state']
    lines += ['', '## Final state (telemetry)', '',
              f'- Posture: {final["posture"]["posture"]} (confident: {final["posture"]["confident"]}); '
              f'leg extensions m: `{_fmt(final["posture"]["extensions_m"])}`',
              f'- At rest: {final["stationary"]}; detail `{_fmt(final["rest_detail"])}`',
              '', '## Evidence notes', '',
              '- Success criteria are telemetry measurements, never command publication.',
              f'- Posture criteria: {report["posture_criteria"]["basis"]}.',
              '- Leg odometry is the vendor estimator; it is not ground truth for distance.',
              '- Simulation and read-only results are software evidence, not physical movement.']
    if report['events']:
        lines += ['', '## Events', ''] + [f'- {_fmt(e)}' for e in report['events']]
    return '\n'.join(lines) + '\n'


def write_reports(report, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / 'report.json'
    md_path = directory / 'report.md'
    json_path.write_text(json.dumps(report, indent=1, sort_keys=False, default=str))
    md_path.write_text(markdown(report))
    return {'json': json_path, 'markdown': md_path}
