# =============================
# BinaryReceiver.py
# @discription  Subscribe core-dump and Output to file system
# @Author       Koki Nagahama (@hamstick)
# =============================

import os
import time

import rclpy
from rclpy.node import Node
from roq_msgsrv.msg import CopiedBinaryMsg

## Debug Utilities
import pprint
import traceback

class BinaryReceiver(Node):
  SELFNODE = 'bin_receiver'
  SELFTOPIC = 'core_path'

  ## core file
  core_dict = {}

  def __init__(self):
    super().__init__(self.SELFNODE)
     self.get_logger().info('{} initializing...'.format(self.SELFNODE))
    self.sub = self.create_subscription(CopiedBinaryMsg, self.SELFTOPIC, self.sub_callback, 10)
    self.get_logger().info('{} do...'.format(self.SELFNODE))
  
  def __del__(self):
    self.get_logger().info('{} done.'.format(self.SELFNODE))

  def write_core(self, core_pid):
    dump_dir = 'core_dumps/'
    os.makedirs(dump_dir, exist_ok = True)
    corename = (dump_dir + 'core.{}.bin'.format(core_pid))
    with open(corename, mode = 'wb') as fp:
      fp.write(self.core_dict[core_pid])
    self.get_logger().info('pid ({}): core was dumped to file'.format(core_pid))
  
  def sub_callback(self, message):
    """
    Discription:
      Checking and storing elements arriving by Subscribe, data frame conversion
      value of valid_flag:
      - 0: OK
      - 1: final binary data
      - 2: NG
    """
    if message.status <= 1:
      if message.pid in self.core_dict:
        for by in message.core_data:
          self.core_dict[message.pid] += by
      else:
        for by in message.core_data:
          self.core_dict[message.pid] = by
      
      self.get_logger().info('PID ({}): binary data received: size: {}, core-size: {}'.format(
        message.pid, len(message.core_data), len(self.core_dict[message.pid])
      ))
      if message.status == 1:
        self.write_core(message.pid)
        self.core_dict[message.pid] = b''


def main(args = None):
  rclpy.init(args = args)
  node = BinaryReceiver()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    print('\nGot Ctrl+C.  System is stopped..')
  except Exception:
    print('\nException raised..  System will be shutdown..')
    traceback.print_exc()
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
