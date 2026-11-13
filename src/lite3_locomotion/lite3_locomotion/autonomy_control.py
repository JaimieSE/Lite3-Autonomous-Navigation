"""Explicit start/stop client; sends only fixed software service requests."""

import argparse
import sys
import time


def main(args=None):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('command', choices=['start', 'stop'])
    options = parser.parse_args(sys.argv[1:] if args is None else args)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from lite3_locomotion.exploration_coordinator import START, MISSION_STOP

    context = Context()
    rclpy.init(args=[], context=context)
    node = Node('lite3_autonomy_control', context=context, use_global_arguments=False,
                enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    def call(service):
        client = node.create_client(Trigger, service)
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('Service unavailable: ' + service)
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            if not future.done():
                raise RuntimeError('Service response timed out: ' + service)
            response = future.result()
            print(service + ': ' + response.message)
            if not response.success:
                raise RuntimeError('Request rejected by ' + service)
        finally:
            node.destroy_client(client)

    try:
        if options.command == 'start':
            call(START)
        else:
            call(MISSION_STOP)
    except Exception as error:
        if options.command == 'start':
            try:
                call(MISSION_STOP)
            except Exception:
                pass
        raise SystemExit(str(error))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
