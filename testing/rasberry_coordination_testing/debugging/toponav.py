#! /usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup as RCG
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty
from topological_navigation_msgs.msg import ExecutePolicyModeGoal

class Nodey(Node):
    def __init__(self):
        super().__init__('nodey2')
        qos = QoSProfile(depth=1)
        self.sub1 = self.create_subscription(ExecutePolicyModeGoal, '/robot_01/topological_navigation/execute_policy_mode/goal', self.cb1, qos, callback_group=RCG())

    def cb1(self, msg):
        print("<<cb1>>")
        print(msg)
        print(">>cb1<<")


def main(args=None):
    rclpy.init(args=args)

    No = Nodey()

    executor = MultiThreadedExecutor()
    executor.add_node(No)
    executor.spin()

    No.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
