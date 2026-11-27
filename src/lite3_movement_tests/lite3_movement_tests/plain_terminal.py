"""Plain-words terminal for competition runs (operator 2026-11-27: "make it such that the terminal prints in terms that
are important and easy to understand").

With `roam run --terminal plain` the terminal shows only:
  * the NOW: activity line when it changes (roam._say_activity),
  * one plain sentence for each important event (this table; every other event is not printed),
  * a short progress summary every SUMMARY_EVERY_S.
Every raw event and the detailed 1 s status line still go to report.json and to <report dir>/terminal_full.log.
Display only: nothing here changes what the robot does.
"""

SUMMARY_EVERY_S = 15.0


def _cell(d, key='cell'):
    c = d.get(key)
    return f'{c[0]},{c[1]}' if isinstance(c, (list, tuple)) and len(c) >= 2 else '?'


def _n(v, fmt='{:.1f}'):
    try:
        return fmt.format(float(v))
    except (TypeError, ValueError):
        return '?'


def _room_result(d):
    bits = []
    if d.get('room'):
        bits.append(f"a {d['room']}")
    finds = d.get('findings')
    if isinstance(finds, list) and finds:
        bits.append('found ' + ', '.join(str(f[1] or f[0]) if isinstance(f, (list, tuple)) else str(f)
                                         for f in finds[:4]))
    elif isinstance(finds, int) and finds:
        bits.append(f'{finds} finding(s)')
    qr = d.get('qr')
    if isinstance(qr, list) and qr:
        bits.append('QR ' + ', '.join(str(q) for q in qr[:2]))
    elif isinstance(qr, int) and qr:
        bits.append(f'{qr} QR code(s)')
    return '; '.join(bits) or 'nothing notable'


SAY = {
    # --- safety and pauses
    'safety_stop': lambda d: f"!! SAFETY STOP - robot stopped: {d.get('reason', '?')}",
    'paused': lambda d: f"!! PAUSED: {d.get('reason', '?')}" + (
        f" (resumes by itself in {_n(d['auto_resume_s'], '{:.0f}')} s)" if d.get('auto_resume_s') else ''),
    'resumed': lambda d: 'Resumed - roaming again',
    'auto_resumed': lambda d: 'Resumed by itself - roaming again',
    'auto_resume_refused': lambda d: f"Not resuming yet: {d.get('reason', '?')} (tries again in "
                                     f"{_n(d.get('retry_s'), '{:.0f}')} s)",
    'tilt_escape': lambda d: f"Tilted - stepping {d.get('direction', '?')} to get level (try {d.get('attempt', '?')})",
    'map_reset': lambda d: f"Map was reset: {d.get('reason', '?')}",
    # --- stairs and blocks
    'stairs_pi_seen': lambda d: 'Pi camera sees stairs ahead',
    'stairs_start': lambda d: f"STAIRS: starting to climb {d.get('direction', '')} ({d.get('mode', '?')} mode)",
    'stairs_done': lambda d: f"STAIRS: done ({d.get('direction', '')}, walked {_n(d.get('travelled_m'))} m)",
    'stairs_aborted': lambda d: f"STAIRS: gave up - {d.get('reason', '?')}; carrying on exploring",
    'step_detected': lambda d: f"Block ahead ({_n(d.get('rise_m', 0) * 100 if d.get('rise_m') else None, '{:.0f}')} cm "
                               f"high, {_n(d.get('ahead_m'))} m away) - crossing it",
    'step_not_crossed': lambda d: f"Not crossing the block yet: {d.get('why', '?')}",
    'on_block': lambda d: 'On top of a block',
    'off_block': lambda d: 'Off the block, back on the floor',
    'curtain_detected': lambda d: f"Curtain ahead (Pi camera, {_n(d.get('ahead_m'))} m) - walking through it",
    'curtain_crossed': lambda d: f"Went through the curtain ({_n(d.get('travelled_m'))} m)",
    'curtain_crossing_failed': lambda d: f"Could not get through the curtain: {d.get('reason', '?')}",
    # --- rooms and pictures
    'room_protocol_start': lambda d: f"Room found at cell {_cell(d)} - reading it from outside",
    'room_view_done': lambda d: (f"Room {_cell(d)}: no reading ({d['failure']})" if d.get('failure')
                                 else f"Room {_cell(d)} read: {_room_result(d)}"),
    'room_scan_done': lambda d: f"Room {_cell(d)} photographed ({d.get('views', '?')} views): "
                                f"{('a ' + d['room']) if d.get('room') else 'type unknown'}",
    'room_protocol_done': lambda d: f"Finished with room {_cell(d)}" + (
        f" ({d['result']})" if isinstance(d.get('result'), str) else ''),
    'room_protocol_skipped': lambda d: f"Skipped room {_cell(d)}: {d.get('reason', d.get('why', '?'))}",
    'capture_done': lambda d: f"Photo of cell {_cell(d)}: {d.get('result') or 'no answer'}",
    'mission': lambda d: f"** FOUND: {str(d.get('kind') or 'something').replace('mission_', '').replace('_', ' ')}"
                         + (f" = {d['value']}" if d.get('value') not in (None, '') else ''),
    'scene_hold': lambda d: f"Holding still for the Pi camera: {d.get('reason', '?')}",
    'pi_obstacle_stop': lambda d: f"Pi camera sees {d.get('obstacle', 'an obstacle')} "
                                  f"{_n(d.get('distance_m'))} m ahead - stopping to check",
    # --- progress and getting stuck
    'stuck_detected': lambda d: f"Stuck: moved only {_n(d.get('moved_m'))} m in {_n(d.get('window_s'), '{:.0f}')} s "
                                f"- trying something else",
    'stuck_resolved': lambda d: 'No longer stuck',
    'walk_failed': lambda d: f"Could not walk there: {d.get('reason', '?')}",
    'goal_failed': lambda d: f"Gave up on that goal: {d.get('reason', '?')}",
    'back_for_room': lambda d: f"No room to turn - walking backwards (up to {_n(d.get('max_m'))} m)",
    'back_for_room_impossible': lambda d: 'No room to turn and no room behind either',
    'walk_mode_resent': lambda d: 'Turn not happening after the room photos - sent walking mode again',
    'auto_mode_resent': lambda d: 'Dog not moving although told to - switched it back to auto (navigation) mode',
    'auto_mode_operator': lambda d: 'Operator asked: robot switched back to auto (navigation) mode',
    'walk_mode_resend_failed': lambda d: f"Could not re-send walking mode: {d.get('error', '?')}",
    'turn_skipped': lambda d: f"Skipped a turn: {d.get('reason', '?')}",
    'arena_hold': lambda d: f"Waiting a moment: {d.get('reason', '?')}",
    'grid_complete': lambda d: 'Every grid cell explored' + (' - checking the map edge' if d.get('frontier') else ''),
    'exit_detected': lambda d: 'Exit spotted',
    'exit_reached': lambda d: '** EXIT REACHED',
    'look_abandoned': lambda d: f"Look-around cut short: {d.get('reason', d.get('why', '?'))}",
}


def say(name, detail):
    """The plain sentence for an event, or None when the event is not worth the operator's attention."""
    fn = SAY.get(name)
    if fn is None:
        return None
    try:
        return fn(detail or {})
    except Exception:  # noqa: BLE001 - a display line must never stop roaming
        return name.replace('_', ' ')


def summary(status, course=None):
    """One short progress line from roam's status dict."""
    s = status.get('stats') or {}
    bits = [f"walked {_n(s.get('distance_m'))} m", f"map {_n(status.get('map_area_m2'), '{:.0f}')} m2"]
    if course is not None:
        try:
            if course.rooms():
                bits.append(f'rooms read {len(course.rooms_read)}/{len(course.rooms())}')
        except Exception:  # noqa: BLE001
            pass
    if s.get('mission_events'):
        bits.append(f"finds {s['mission_events']}")
    hz = status.get('lidar_hz')
    if hz is not None:
        bits.append(f'LiDAR {_n(hz)} Hz')
    return 'Progress: ' + ', '.join(bits)
