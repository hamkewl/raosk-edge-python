#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
# -----------------------------------------------
# ROS2 Node 送信側
#
# The MIT License (MIT)
# Copyright (C) 2019 myasu.
# -----------------------------------------------

import os

import rclpy
from rclpy.node import Node

import random
from roq_msgsrv.msg import NwProcMsg

class MyPublisherNw(Node):
	SELFNODE = "testpub2"
	SELFTOPIC = "nw_proc"
	boundary = 0.025
	vgid = os.getppid()

	def __init__(self):
		super().__init__(self.SELFNODE)
		self.get_logger().info("{} initializing...".format((self.SELFNODE)))
		self.pub = self.create_publisher(NwProcMsg, self.SELFTOPIC, 10)
		self.create_timer(1.00, self.callback)
		self.get_logger().info("{} do...".format(self.SELFNODE))
		self.count = 0

	def __del__(self):
		self.get_logger().info("{} done.".format(self.SELFNODE))

	def callback(self):
		msg = NwProcMsg()

		p = random.random()
		self.count += 1

		if self.count >= 90 and p <= self.boundary or self.count >= 360:
			msg.is_valid = 1
		elif (1. - self.boundary) <= p:
			msg.is_valid = 2
		else:
			msg.is_valid = 0
		
		msg.n_send = 1 + random.randint(0, 114514)
		msg.n_receive = 1 + random.randint(0, 810)

		self.get_logger().info("Publish [{:3d}] --> (is_valid = {}))".format(self.count, msg.is_valid))
		self.pub.publish(msg)

		if self.count >= 360 or self.count >= 90 and msg.is_valid == 1:
			self.get_logger().info('Tester will be stopeed..')
			raise KeyboardInterrupt

def main(args = None):
	rclpy.init(args = args)
	node = MyPublisherNw()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
