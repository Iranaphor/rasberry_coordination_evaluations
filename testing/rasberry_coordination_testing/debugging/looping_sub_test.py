#! /usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup as RCG
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty

class Nodey(Node):
    def __init__(self):
        super().__init__('nodey2')
        qos = QoSProfile(depth=3)

        self.pub1 = self.create_publisher(Empty, '~/cb1', qos)
        self.sub1 = self.create_subscription(Empty, '~/cb1', self.cb1, qos, callback_group=RCG())

        self.pub0 = self.create_publisher(Empty, '~/spin', qos)
        self.sub0 = self.create_subscription(Empty, '~/spin', self.spin, qos, callback_group=RCG())
        self.pub0.publish(Empty())

    def spin(self, msg):
        print("--spin--")
        self.pub1.publish(Empty())
        while True:
            time.sleep(1)
        print("__spin__")

    def cb1(self, msg):
        print("<<cb1>>")
        time.sleep(1)
        self.pub1.publish(Empty())
        time.sleep(5)
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
