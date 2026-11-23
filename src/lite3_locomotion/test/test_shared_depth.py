"""sensor_relay shares the robot's raw depth frames on this PC (one robot-link copy for all consumers)."""
import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from lite3_locomotion import sensor_relay as S


class FakeNode:
    def __init__(self):
        self.pubs, self.subs = {}, {}

    def create_publisher(self, msg_type, topic, qos):
        pub = type('Pub', (), {})()
        pub.sent = []
        pub.publish = pub.sent.append
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos, **kw):
        self.subs[topic] = callback

    def get_logger(self):
        return type('L', (), {'info': lambda self, *a: None})()


def test_raw_depth_and_info_are_republished_unchanged_and_points_still_made():
    src, out = FakeNode(), FakeNode()
    S._depth_relay(src, out, {}, {}, {})
    assert S.SHARED_DEPTH_IMAGE == '/lite3/camera/depth/image_raw'
    assert S.SHARED_DEPTH_INFO == '/lite3/camera/depth/camera_info'
    info = CameraInfo(width=640, height=480, k=[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0])
    info.header.frame_id = 'camera_depth_optical_frame'
    src.subs[S.DEPTH_INFO](info)
    depth = Image(width=640, height=480, encoding='16UC1', step=1280,
                  data=np.full((480, 640), 800, np.uint16).tobytes())
    depth.header.frame_id = 'camera_depth_optical_frame'
    src.subs[S.DEPTH_SOURCE](depth)
    assert out.pubs[S.SHARED_DEPTH_INFO].sent == [info]
    assert out.pubs[S.SHARED_DEPTH_IMAGE].sent == [depth]            # the very same message, unchanged
    assert len(out.pubs[S.DEPTH_OUTPUT].sent) == 1                   # terrain points as before


def test_frames_without_intrinsics_are_still_shared():
    src, out = FakeNode(), FakeNode()
    S._depth_relay(src, out, {}, {}, {})
    frame = Image(width=4, height=2, encoding='16UC1', step=8, data=bytes(16))
    src.subs[S.DEPTH_SOURCE](frame)                                  # no camera_info yet
    assert out.pubs[S.SHARED_DEPTH_IMAGE].sent == [frame] and out.pubs[S.DEPTH_OUTPUT].sent == []
