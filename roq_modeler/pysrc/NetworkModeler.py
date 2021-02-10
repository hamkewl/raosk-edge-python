# =============================
# NetworkModeler.py
# @discription  Modeling of acquired network data and sending/receiving via ROS2
# @Author       Koki Nagahama (@hamstick)
# =============================

## Regression Kit
import time
import numpy as np

## ROS2
import rclpy
from rclpy.node import Node
from roq_msgsrv.msg import NwProcMsg, NwParamsMsg	## Subscribe, Publish

## Debug Utilities
import pprint
import traceback

class NetworkModeler(Node):
  ## Node & Topic Name
  NODENAME = 'nw_modeler'
  PUBTOPIC = 'nw_params'
  SUBTOPIC = 'nw_proc'
  r_INTERVAL = 60

  ## Instances
  exec_time = []

  ## Model Parameters
  #MID = -1
  p_ave_send = 0.00
  p_ave_receive = 0.00
  
  ## single robot version
  elements_of_X = [
    'n_send',
    'n_receive'
  ]
  element_of_Y = ['n_load']
  nw_data_tuple = np.empty(len(elements_of_X))
  is_init = True

  def __init__(self):
    super().__init__(self.NODENAME)
    self.get_logger().info('{} initializing...'.format(self.NODENAME))

    ## Create ROS2 instance
    self.pub = self.create_publisher(NwParamsMsg, self.PUBTOPIC, 10)
    self.timer = self.create_timer(1.00, self.modeler_pub_callback)
    self.sub = self.create_subscription(NwProcMsg, self.SUBTOPIC, self.modeler_sub_callback, 10)
  
  def __del__(self):
    self.get_logger().info("{} done.".format(self.NODENAME))
    npa = np.array(self.exec_time)
    print('(Publish)   data size[] = {}, mean([]): {:.6f}, min([]): {:.6f}, max([]): {:.6f}, std([]): {:.6f}'.format(
      len(npa), np.mean(npa), np.min(npa), np.max(npa), np.std(npa)
    ))
  
  ## Create model
  def create_params(self, ndarray):
    self.p_ave_send = np.mean(ndarray[:, 0])
    self.p_ave_receive = np.mean(ndarray[:, 1])
    self.get_logger().info('create_params:  p_ave_send: {:.4f},  p_ave_receive: {:.4f}'.format(
      self.p_ave_send, self.p_ave_receive
    ))
  
  ## callback function when publish message
  def modeler_pub_callback(self):
    ## Message setting
    msg = NwParamsMsg()
    msg.p_ave_send = self.p_ave_send
    msg.p_ave_receive = self.p_ave_receive

    ## Send message
    self.pub.publish(msg)
  
  ## callback function when subscribe message
  """
    Discription:
      Checking and storing elements arriving by Subscribe, data frame conversion
      value of valid_flag:
      - 0: OK
      - 1: Process killed
      - 2: NG
  """
  def modeler_sub_callback(self, message):
    start = time.time()
    valid_flag = message.is_valid

    if valid_flag <= 1:
      nw_data_arrival = np.array(
        [
          message.n_send,
          message.n_receive
        ]
      )
      if not(len(nw_data_arrival) == len(self.elements_of_X)):
        raise ValueError
      else:
        if self.is_init == True:
          self.nw_data_tuple = np.vstack( (self.nw_data_tuple, nw_data_arrival) )
          self.nw_data_tuple = np.delete(self.nw_data_tuple, 0, axis = 0)
          self.is_init = False
        else:
          self.nw_data_tuple = np.vstack( (self.nw_data_tuple, nw_data_arrival) )

        """
          Run multiple regression when r_INTERVAL data is collected
          or when there is more than 10 times the minimum number of parameters when valid_flag is 1
        """
        if len(self.nw_data_tuple) >= self.r_INTERVAL \
          or valid_flag == 1 and len(self.nw_data_tuple) >= (len(self.elements_of_X) * 10):

          ## Reinitialize nw_data_tuple
          ndarray = self.nw_data_tuple
          self.nw_data_tuple = np.empty(len(self.elements_of_X))
          self.is_init = True

          ## Multiple regression model building
          self.create_params(ndarray)
        
        # else
        elif valid_flag == 1:
          self.get_logger().warn('Executing Modeling skipped.. (self.nw_data_tuple: {})'.format(len(self.nw_data_tuple)))

    else:
      self.get_logger().warn('Invalid data received.')
    
    end = time.time()
    self.exec_time.append(end - start)
    self.get_logger().info('nw_data_tuple: {:2d}, valid_flag: {}, raptime: {:.4f}'.format(
      len(self.nw_data_tuple), valid_flag, end - start)
    )


def main(args = None):
  rclpy.init(args = args)
  node = NetworkModeler()
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
