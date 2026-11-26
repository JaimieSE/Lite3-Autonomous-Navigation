"""Bounded frontier missions consuming existing Nav2; preview by default.

No velocity publisher, transport, lifecycle transitions or mode commands.
Enabling action dispatch can move a robot through its existing navigation stack.
"""

import argparse
import json
import math
import re
import sys
import time

from lite3_locomotion.exploration_preview import grid_from_message
from lite3_locomotion.frontiers import FrontierConfig, select_frontiers
from lite3_locomotion.safety import scan_is_clear


ACTION = '/navigate_to_pose'
START = '/lite3_locomotion/start_exploration'
MISSION_STOP = '/lite3_locomotion/stop_exploration'


def parse_options(args):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--radius', type=float, default=3.0)
    parser.add_argument('--session-seconds', type=float, default=60.0)
    parser.add_argument('--nav2-namespace', default='')
    parser.add_argument('--enable-navigation', action='store_true',
                        help='Permit goals to the existing Nav2 server; may cause physical motion')
    options = parser.parse_args(args)
    try:
        FrontierConfig(radius=options.radius, clearance=0.4)
        if not math.isfinite(options.session_seconds) or not 0 < options.session_seconds <= 300:
            raise ValueError('session-seconds must be in (0, 300]')
        namespace = options.nav2_namespace.strip('/')
        if namespace and any(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', part) is None
                             for part in namespace.split('/')):
            raise ValueError('Invalid Nav2 namespace')
        options.nav2_namespace = '/' + namespace if namespace else ''
    except ValueError as error:
        parser.error(str(error))
    return options


def create_node(options, context):
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener

    class Coordinator(Node):
        def __init__(self):
            super().__init__('lite3_exploration_coordinator', namespace='/', context=context,
                             use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
            self.running = False
            self.epoch = 0
            self.grid = self.map_received = None
            self.scan_received = self.scan_stamp = None
            self.scan_clear = False
            self.started_at = None
            self.inflight = {}
            self.cancel_deadlines = {}
            self.cancel_failed = False
            self.center = None
            self.goal_handle = None
            self.pending = False
            self.sent_at = None
            self.current = None
            self.blocked = {}
            self.failures = self.goals_sent = 0
            self.reason = 'idle'
            self.lifecycle_states = {}
            self.lifecycle_pending = set()
            self.lifecycle_clients = {name: self.create_client(
                GetState, options.nav2_namespace + '/' + name + '/get_state')
                for name in ('controller_server', 'planner_server', 'bt_navigator')}
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.status_pub = self.create_publisher(String, '/lite3_locomotion/exploration_status', qos)
            # Preview cannot submit a goal even if a navigation server is present.
            self.client = (ActionClient(self, NavigateToPose, options.nav2_namespace + ACTION)
                           if options.enable_navigation else None)
            self.start_service = self.create_service(Trigger, START, self.start)
            self.stop_service = self.create_service(Trigger, MISSION_STOP, self.stop_service_callback)
            self.timer = self.create_timer(2.0, self.tick)
            self.watchdog = self.create_timer(0.1, self.supervise)
            self.lifecycle_timer = self.create_timer(0.5, self.check_lifecycles)

        def check_lifecycles(self):
            for name, client in self.lifecycle_clients.items():
                if name not in self.lifecycle_pending and client.service_is_ready():
                    self.lifecycle_pending.add(name)
                    future = client.call_async(GetState.Request())
                    future.add_done_callback(lambda done, name=name: self.lifecycle_result(done, name))

        def lifecycle_result(self, future, name):
            self.lifecycle_pending.discard(name)
            try:
                self.lifecycle_states[name] = (future.result().current_state.id, time.monotonic())
            except Exception:
                self.lifecycle_states.pop(name, None)

        def on_map(self, message):
            try:
                self.grid = grid_from_message(message)
                self.map_received = time.monotonic()
            except (ValueError, TypeError, OverflowError):
                self.grid = None
                if self.running:
                    self.stop_mission('invalid_map')

        def on_scan(self, message):
            self.scan_stamp = Time.from_msg(message.header.stamp)
            coverage = abs(message.angle_increment) * max(0, len(message.ranges) - 1)
            self.scan_clear = (message.header.frame_id == 'base_link' and
                               math.isfinite(coverage) and 6.0 <= coverage <= 6.4 and
                               scan_is_clear(message.ranges, message.range_min, message.range_max))
            self.scan_received = time.monotonic()
            if self.running and not self.scan_clear:
                self.stop_mission('scan_obstacle_or_invalid')

        def ready(self):
            now = time.monotonic()
            scan_age = ((self.get_clock().now() - self.scan_stamp).nanoseconds / 1e9
                        if self.scan_stamp is not None else math.inf)
            return (self.scan_clear and self.scan_received is not None and
                    0 <= now - self.scan_received <= 0.5 and -0.2 <= scan_age <= 0.5 and
                    self.grid is not None and self.map_received is not None and now - self.map_received <= 30 and
                    (self.client is None or (self.nav2_active() and self.client.server_is_ready())))

        def nav2_active(self):
            now = time.monotonic()
            return all(name in self.lifecycle_states and self.lifecycle_states[name][0] == 3 and
                       now - self.lifecycle_states[name][1] <= 2.0 for name in self.lifecycle_clients)

        def pose(self):
            try:
                transform = self.buffer.lookup_transform('map', 'base_link', Time())
                age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                p = transform.transform.translation
                if -0.2 <= age <= 0.5 and all(math.isfinite(v) for v in (p.x, p.y)):
                    return p.x, p.y
            except TransformException:
                pass
            return None

        def start(self, request, response):
            p = self.pose()
            if self.running:
                response.success, response.message = True, 'Mission already running; bounds unchanged'
                return response
            if self.inflight or self.cancel_failed:
                response.success, response.message = False, 'Previous goal termination is unconfirmed'
                return response
            if not self.ready() or p is None:
                response.success = False
                response.message = 'Requires fresh map/TF/clear scan and, for dispatch, active existing Nav2'
                return response
            self.epoch += 1
            self.running = True
            self.center = p
            self.started_at = time.monotonic()
            self.current = None
            self.failures = self.goals_sent = 0
            self.blocked.clear()
            self.reason = 'running'
            response.success, response.message = True, ('Bounded frontier mission started' if self.client
                                                        else 'Preview started; navigation output disabled')
            self.tick()
            return response

        def stop_service_callback(self, request, response):
            self.stop_mission('operator_stop')
            response.success, response.message = True, 'Mission stopped; goal cancellation requested, not physical stop confirmation'
            return response

        def stop_mission(self, reason):
            self.running = False
            self.epoch += 1  # Late goal-accept callbacks must cancel, never resume.
            self.reason = reason
            for epoch, handle in list(self.inflight.items()):
                if handle is not None:
                    self.cancel_goal(epoch, handle)
            self.goal_handle = None
            self.pending = False
            self.sent_at = None

        def cancel_goal(self, epoch, handle):
            if epoch in self.cancel_deadlines:
                return
            self.cancel_deadlines[epoch] = time.monotonic() + 3.0
            try:
                handle.cancel_goal_async().add_done_callback(lambda done: self.cancel_response(done, epoch))
            except Exception:
                self.cancel_failed = True

        def cancel_response(self, future, epoch):
            if epoch not in self.inflight:
                return  # Terminal result already received.
            try:
                response = future.result()
                if response.return_code != 0 or not response.goals_canceling:
                    self.cancel_failed = True
            except Exception:
                self.cancel_failed = True

        def supervise(self):
            if self.running and (not self.ready() or self.pose() is None):
                self.stop_mission('map_tf_scan_or_nav2_unavailable')
            if self.running and (time.monotonic() - self.started_at >= options.session_seconds or
                                 math.dist(self.pose() or self.center, self.center) >= options.radius):
                self.stop_mission('session_deadline_or_radius')
            if self.running and self.sent_at is not None and time.monotonic() - self.sent_at >= 30:
                self.stop_mission('goal_deadline')
            if any(time.monotonic() >= deadline for deadline in self.cancel_deadlines.values()):
                self.cancel_failed = True
            if self.cancel_failed:
                self.reason = 'goal_termination_unconfirmed'

        def tick(self):
            message = String()
            message.data = json.dumps(dict(running=self.running, reason=self.reason, goals_sent=self.goals_sent,
                                          nav2_active=self.nav2_active(), navigation_enabled=self.client is not None,
                                          pending_termination=bool(self.inflight) and not self.running,
                                          selected_goal=(dict(x=self.current.x, y=self.current.y, yaw=self.current.yaw)
                                                         if self.current else None)))
            self.status_pub.publish(message)
            if not self.running or self.pending or self.goal_handle is not None:
                return
            p = self.pose()
            if not self.ready() or p is None:
                self.stop_mission('inputs_unavailable')
                return
            if self.goals_sent >= 20:
                self.stop_mission('goal_budget_complete')
                return
            result = select_frontiers(self.grid, *p, FrontierConfig(radius=options.radius, clearance=0.4))
            now = time.monotonic()
            self.blocked = {key: until for key, until in self.blocked.items() if until > now}
            choices = [c for c in result.candidates if (
                math.dist((c.x, c.y), self.center) < options.radius - 0.4 and
                math.dist((c.x, c.y), p) >= 0.5 and self.key(c) not in self.blocked)]
            if not choices:
                self.stop_mission('no_eligible_frontiers_in_session_area')
                return
            self.current = choices[0]
            if self.client is None:
                self.reason = 'preview_goal_selected'
                return
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = self.current.x, self.current.y
            goal.pose.pose.orientation.z = math.sin(self.current.yaw / 2)
            goal.pose.pose.orientation.w = math.cos(self.current.yaw / 2)
            self.pending = True
            self.sent_at = now
            self.goals_sent += 1
            epoch = self.epoch
            self.inflight[epoch] = None
            try:
                future = self.client.send_goal_async(goal)
                future.add_done_callback(lambda done: self.accepted(done, epoch))
            except Exception:
                self.cancel_failed = True
                self.stop_mission('goal_submission_unconfirmed')

        @staticmethod
        def key(candidate):
            return round(candidate.x / 0.5), round(candidate.y / 0.5)

        def accepted(self, future, epoch):
            try:
                handle = future.result()
                accepted = handle.accepted
            except Exception:
                self.cancel_failed = True
                if epoch == self.epoch:
                    self.stop_mission('goal_submission_unconfirmed')
                return
            if not accepted:
                self.inflight.pop(epoch, None)
                if epoch == self.epoch and self.running:
                    self.pending = False
                    self.finish_goal(False)
                return
            self.inflight[epoch] = handle
            # Set state before registering callbacks: a completed future may
            # invoke its callback immediately and submit the next frontier.
            if epoch == self.epoch and self.running:
                self.pending = False
                self.goal_handle = handle
            try:
                handle.get_result_async().add_done_callback(lambda done: self.completed(done, epoch))
            except Exception:
                self.cancel_failed = True
                self.stop_mission('goal_result_subscription_failed')
                return
            if (epoch != self.epoch or not self.running) and epoch in self.inflight:
                self.cancel_goal(epoch, handle)

        def completed(self, future, epoch):
            try:
                status = future.result().status
                if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                    raise ValueError('Nonterminal navigation result')
            except Exception:
                self.cancel_failed = True
                self.stop_mission('goal_result_unconfirmed')
                return
            self.inflight.pop(epoch, None)
            self.cancel_deadlines.pop(epoch, None)
            if epoch == self.epoch and self.running:
                self.finish_goal(status == GoalStatus.STATUS_SUCCEEDED)

        def finish_goal(self, succeeded):
            if self.current is not None:
                self.blocked[self.key(self.current)] = time.monotonic() + (30 if succeeded else 60)
            self.goal_handle = None
            self.pending = False
            self.sent_at = None
            self.failures = 0 if succeeded else self.failures + 1
            if self.failures >= 3:
                self.stop_mission('repeated_goal_failure')
            elif self.running:
                self.tick()  # Replan promptly rather than waiting idle for the periodic timer.

    return Coordinator()


def main(args=None):
    options = parse_options(sys.argv[1:] if args is None else args)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = create_node(options, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None and context.ok():
                node.stop_mission('shutdown')
                deadline = time.monotonic() + 3.0
                while executor is not None and node.inflight and context.ok() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                if node.inflight:
                    node.get_logger().error('Navigation termination unconfirmed during shutdown')
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
