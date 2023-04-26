#! /usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty

class Nodey(Node):
    def __init__(self):
        super().__init__('nodey')
        qos = QoSProfile(depth=2)
        sub1 = self.create_subscription(Empty, '/cb1', self.cb1, qos, callback_group=ReentrantCallbackGroup())
        sub2 = self.create_subscription(Empty, '/cb2', self.cb2, qos, callback_group=ReentrantCallbackGroup())
        
    def cb1(self, msg):
        print("<<cb1>>")
        time.sleep(10)
        print(">>cb1<<")

    def cb2(self, msg):
        print("[[cb2]]")
        time.sleep(1)
        print("]]cb2[[")
        

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
