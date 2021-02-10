# =============================
# tester_mem.py
# @discription  Python blog for testing MemoryModeler.py
# @Author       Koki Nagahama (@hamstick)
# =============================

import os

import rclpy
from rclpy.node import Node

import random
from roq_msgsrv.msg import MemProcMsg

class MyPublisherMem(Node):
  SELFNODE = "testpub1"
  SELFTOPIC = "mem_proc"
  boundary = 0.025
  vgid = os.getppid()

  def __init__(self):
    super().__init__(self.SELFNODE)
    self.get_logger().info("{} initializing...".format((self.SELFNODE)))
    self.pub = self.create_publisher(MemProcMsg, self.SELFTOPIC, 10)
    self.create_timer(1.00, self.callback)
    self.get_logger().info("{} do...".format(self.SELFNODE))
    self.count = 0

  def __del__(self):
    self.get_logger().info("{} done.".format(self.SELFNODE))

  def callback(self):
    msg = MemProcMsg()

    p = random.random()
    self.count += 1
    msg.vgid = self.vgid

    if self.count >= 90 and p <= self.boundary or self.count >= 360:
      msg.is_valid = 1
    elif (1. - self.boundary) <= p:
      msg.is_valid = 2
    else:
      msg.is_valid = 0
    
    msg.system = 60.5000 + random.uniform(0, 4.5000)
    msg.buffer_sz = 308 + random.uniform(-50, 50)
    msg.cache_sz = 400 + random.uniform(-50, 50)
    msg.heap_sz = 1000 + random.uniform(0, 200)
    msg.stack_sz = 40 + random.uniform(0, 40)

    self.get_logger().info("Publish [{:3d}] --> (is_valid = {}, p = {:.4f})".format(self.count, msg.is_valid, p))
    self.pub.publish(msg)

    if self.count >= 360 or self.count >= 90 and msg.is_valid == 1:
      self.get_logger().info('Tester will be stopeed..')
      raise KeyboardInterrupt

def main(args = None):
  rclpy.init(args = args)
  node = MyPublisherMem()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
