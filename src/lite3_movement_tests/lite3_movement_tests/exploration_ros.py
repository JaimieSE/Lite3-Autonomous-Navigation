"""ROS side of the exploration record: topic names and MarkerArray conversion (exploration_record.build_markers)."""

import math

STATUS_MARKERS_TOPIC = '/exploration/status_markers'
INSPECTION_REQUESTS_TOPIC = '/exploration/inspection_requests'
CAMERA_CONFIRMATIONS_TOPIC = '/exploration/camera_confirmations'
MAP_FRAME = 'map'

_TYPES = {'arrow': 0, 'cube': 1, 'sphere': 2, 'cylinder': 3, 'line_strip': 4, 'line_list': 5, 'cube_list': 6,
          'sphere_list': 7, 'text': 9}


def markers_to_msg(specs, stamp, frame=MAP_FRAME):
    """visualization_msgs/MarkerArray: a DELETEALL first (so markers no longer in the record disappear, whatever
    their namespace or id), then every spec as an ADD."""
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    out = MarkerArray()
    clear = Marker()
    clear.header.frame_id, clear.header.stamp = frame, stamp
    clear.action = Marker.DELETEALL
    out.markers.append(clear)
    for s in specs:
        m = Marker()
        m.header.frame_id, m.header.stamp = frame, stamp
        m.ns, m.id, m.type, m.action = s['ns'], int(s['id']), _TYPES[s['type']], Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(s['x']), float(s['y']), float(s['z'])
        m.pose.orientation.z, m.pose.orientation.w = math.sin(s['yaw'] / 2), math.cos(s['yaw'] / 2)
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in s['scale'])
        r, g, b, a = s['color']
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        m.text = s['text']
        m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in s['points']]
        m.frame_locked = True
        out.markers.append(m)
    return out
