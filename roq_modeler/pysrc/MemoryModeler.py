# =============================
# MemoryModeler.py
# @discription	Modeling of acquired data and sending/receiving via ROS2
# @Author	Koki Nagahama (@hamstick)
# =============================

## Regression Kit
import time
import numpy as np
import pandas as pd
import sklearn
from sklearn import linear_model
from concurrent.futures import ThreadPoolExecutor

## ROS2
import rclpy
from rclpy.node import Node
from roq_messrv.msg import MemProcMsg, ModelParametersMsg	## Subscribe, Publish

## Debug Utilities
import pprint
import traceback

class MemoryModeler(Node):
	## Node & Topic Name
	NODENAME = 'mem_modeler'
	PUBTOPIC = 'model_params'
	SUBTOPIC = 'memproc_data'
	r_INTERVAL = 90

	## Instances
	thread_list = []
	#executor = ThreadPoolExecutor(max_workers = 8)

	## Model Parameters
	#MID = -1
	p_buffer = 0.00
	p_cache = 0.00
	p_heap_bringup, p_heap_teleop = 0.00, 0.00
	p_stack_bringup, p_stack_teleop = 0.00, 0.00
	p_intercept = 0.00
	
	## single robot version
	elements_of_X = [
		'buffer_szMB', \
		'cache_szMB', \
		'heap_bringup_szMB', 'heap_teleop_szMB', \
		'stack_bringup_szMB', 'stack_teleop_szMB']
	element_of_Y = ['system_mem_per']
	mem_data_block = np.empty(len(elements_of_X) + len(element_of_Y))
	is_init = True

	def __init__(self):
		super().__init__(self.NODENAME)
		self.get_logger().info('{} initializing...'.format(self.NODENAME))

		## Create ROS2 instance
		self.pub = self.create_publisher(ModelParametersMsg, self.PUBTOPIC, 10)
		self.timer = self.create_timer(1.00, self.modeler_pub_callback)
		self.sub = self.create_subscription(MemProcMsg, self.SUBTOPIC, self.modeler_sub_callback, 10)
	
	def __del__(self):
		self.get_logger().info("%s done." % self.NODENAME)

	## Multi Linear Regression @Multi-Threading
	def regression_part(self, dataframe):
		#self.get_logger().info('into REG')
		clf = linear_model.LinearRegression()
		clf.fit(dataframe[self.elements_of_X], dataframe[self.element_of_Y])

		## Setting message value
		predict_params = clf.coef_[0]
		self.get_logger().info('--> predict_params: {}'.format(
			np.round(np.append(clf.coef_[0], clf.intercept_), decimals = 4))
		)
		self.p_buffer = predict_params[0]
		self.p_cache = predict_params[1]
		self.p_heap_bringup, self.p_heap_teleop = predict_params[2], predict_params[3]
		self.p_stack_bringup, self.p_stack_teleop = predict_params[4], predict_params[5]
		self.p_intercept = clf.intercept_[0]

	## callback function when publish message
	def modeler_pub_callback(self):
		## Message setting
		msg = ModelParametersMsg()
		#msg.machine_id = self.MID
		msg.p_buffer = self.p_buffer
		msg.p_cache = self.p_cache
		msg.p_heap_bringup, msg.p_heap_teleop	= self.p_heap_bringup, self.p_heap_teleop
		msg.p_stack_bringup, msg.p_stack_teleop = self.p_stack_bringup, self.p_stack_teleop
		msg.p_intercept = self.p_intercept

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
		executor = ThreadPoolExecutor(max_workers = 2)
		valid_flag = message.is_valid
		if valid_flag <= 1:
			#self.MID = message.machine_id
			mem_data_arrival = np.array(
				[
					message.system,
					message.buffer_sz,
					message.cache_sz,
					message.heap_bringup_sz, message.heap_teleop_sz,
					message.stack_bringup_sz, message.stack_teleop_sz
				]
			)
			if not(len(mem_data_arrival) == len(self.elements_of_X) + len(self.element_of_Y)):
				raise ValueError
			else:
				if self.is_init == True:
					self.mem_data_block = np.vstack( (self.mem_data_block, mem_data_arrival) )
					self.mem_data_block = np.delete(self.mem_data_block, 0, axis = 0)
					self.is_init = False
				else:
					self.mem_data_block = np.vstack( (self.mem_data_block, mem_data_arrival) )
				
				self.get_logger().info('mem_data_block: {}, valid_flag: {}'.format(len(self.mem_data_block), valid_flag))
				if len(self.mem_data_block) >= self.r_INTERVAL or valid_flag == 1:
	
					dataframe = pd.DataFrame(self.mem_data_block)
					dataframe.columns = self.element_of_Y + self.elements_of_X

					## Reinitialize mem_data_block
					self.mem_data_block = np.empty(len(self.elements_of_X) + len(self.element_of_Y))
					self.is_init = True

					## Multiple regression model building
					self.thread_list.append( executor.submit(self.regression_part, dataframe) )

		else:
			raise ValueError
		end = time.time()
		self.get_logger().info('rap time: {:.4f}'.format(end - start))


def main(args = None):
	rclpy.init(args = args)
	node = MemoryModeler()
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
