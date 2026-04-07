#!/usr/bin/python3
 

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from rmoss_interfaces.srv import ControlTask

banner = """
camera task manager client test
Please input keys:
---------------------------
q. start, e:stop
---------------------------
CTRL-C to quit
"""


def start_req(cli):
    req = ControlTask.Request()
    req.cmd = req.START
    cli.call_async(req)


def stop_req(cli):
    req = ControlTask.Request()
    req.cmd = req.STOP
    cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = Node('virtual_client')
    cli = node.create_client(ControlTask, 'virtual_image_cam/control_task')
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    print(banner)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    while rclpy.ok():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key = sys.stdin.read(1)
            print(key)
            if (key == 'q' or key == 'Q'):
                start_req(cli)
            elif (key == 'w' or key == 'W'):
                stop_req(cli)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
