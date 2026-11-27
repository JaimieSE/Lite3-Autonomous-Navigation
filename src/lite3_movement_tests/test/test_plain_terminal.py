"""Plain-words terminal (operator 2026-11-27)."""
from lite3_movement_tests import plain_terminal as pt


def test_important_events_get_a_plain_sentence():
    assert pt.say('safety_stop', {'reason': 'tilt'}).startswith('!! SAFETY STOP')
    assert 'kitchen' in pt.say('room_view_done', {'cell': [2, 3], 'room': 'kitchen', 'findings': [], 'qr': []})
    assert '20 cm' in pt.say('step_detected', {'ahead_m': 0.5, 'rise_m': 0.2})


def test_routine_events_are_not_printed():
    assert pt.say('walk', {}) is None
    assert pt.say('slow_tick', {'seconds': 0.3}) is None


def test_a_broken_detail_never_raises():
    assert pt.say('step_detected', {'rise_m': 'x'})
    assert pt.summary({}).startswith('Progress:')
