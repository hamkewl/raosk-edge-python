#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS2 Node 送信側
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import rclpy
from rclpy.node import Node

import random
from roq_msgsrv.msg import MemProcMsg

class MyPublisher(Node):
	SELFNODE = "testpub"
	SELFTOPIC = "memproc_data"

	def __init__(self):
		super().__init__(self.SELFNODE)
		self.get_logger().info("%s initializing..." % (self.SELFNODE))
		self.pub = self.create_publisher(MemProcMsg, self.SELFTOPIC, 10)
		self.create_timer(1.00, self.callback)
		self.get_logger().info("%s do..." % self.SELFNODE)
		self.count = 1

	def __del__(self):
		self.get_logger().info("%s done." % self.SELFNODE)

	def callback(self):
		self.get_logger().info("Publish [%s]" % (self.count))
		msg = MemProcMsg()
		
		msg.is_valid = 0
		msg.system = 60.5000 + random.uniform(0, 4.5000)
		msg.buffer_sz = 308 + random.randint(-50, 50)
		msg.cache_sz = 400 + random.randint(-50, 50)
		msg.heap_sz = 1000 + random.randint(0, 200)
		msg.stack_sz = 40 + random.randint(0, 40)
		"""
		msg.heap_bringup_sz = 1000 + random.randint(0, 200)
		msg.heap_teleop_sz = 200 + random.randint(0, 50)
		msg.stack_bringup_sz = 8
		msg.stack_teleop_sz = 40
		"""

		self.pub.publish(msg)
		self.count += 1

def main(args = None):
	try:
		rclpy.init(args = args)
		node = MyPublisher()
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
