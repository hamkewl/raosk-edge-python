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
from roq_msgsrv.msg import MemProcMsg, MemParamsMsg	## Subscribe, Publish

## Debug Utilities
import pprint
import traceback

class MemoryModeler(Node):
	## Node & Topic Name
	NODENAME = 'mem_modeler'
	PUBTOPIC = 'mem_params'
	SUBTOPIC = 'memproc_data'
	r_INTERVAL = 90

	## Instances
	thread_list = []
	#executor = ThreadPoolExecutor(max_workers = 8)

	## Model Parameters
	#MID = -1
	p_vgid = 0
	p_buffer = 0.00
	p_cache = 0.00
	p_heap = 0.00
	p_stack = 0.00
	p_intercept = 0.00
	
	## single robot version
	elements_of_X = [
		'buffer_szMB',
		'cache_szMB',
		'heap_szMB',	#'heap_bringup_szMB', 'heap_teleop_szMB', 		# 単一構成．単一ノードとして実装
		'stack_szMB'	#'stack_bringup_szMB', 'stack_teleop_szMB'		# 単一構成・単一ノードとして実装
  ]
	element_of_Y = ['system_mem_per']
	mem_data_block = np.empty(len(elements_of_X) + len(element_of_Y))
	is_init = True

	def __init__(self):
		super().__init__(self.NODENAME)
		self.get_logger().info('{} initializing...'.format(self.NODENAME))

		## Create ROS2 instance
		self.pub = self.create_publisher(MemParamsMsg, self.PUBTOPIC, 10)
		self.timer = self.create_timer(1.00, self.modeler_pub_callback)
		self.sub = self.create_subscription(MemProcMsg, self.SUBTOPIC, self.modeler_sub_callback, 10)
	
	def __del__(self):
		self.get_logger().info("{} done.".format(self.NODENAME))

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
		self.p_heap = predict_params[2]
		self.p_stack = predict_params[3]
		self.p_intercept = clf.intercept_[0]
		self.get_logger().info('--> clf.score: {:.4f}'.format(
			clf.score(dataframe[self.elements_of_X], dataframe[self.element_of_Y[0]])
		))

	## callback function when publish message
	def modeler_pub_callback(self):
		## Message setting
		msg = MemParamsMsg()
		#msg.machine_id = self.MID
		msg.vgid = self.p_vgid
		msg.p_buffer = self.p_buffer
		msg.p_cache = self.p_cache
		msg.p_heap = self.p_heap
		msg.p_stack = self.p_stack
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
		self.p_vgid = message.vgid
		valid_flag = message.is_valid

		if valid_flag <= 1:
			#self.MID = message.machine_id
			mem_data_arrival = np.array(
				[
					message.system,
					message.buffer_sz,
					message.cache_sz,
					message.heap_sz,
					message.stack_sz
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

				## r_INTERVAL個データが集まったら
				## またはvalid_flagが1のときに最低限パラメータ数の10倍以上データがあったら重回帰
				if len(self.mem_data_block) >= self.r_INTERVAL \
					or valid_flag == 1 and len(self.mem_data_block) >= (len(self.elements_of_X) * 10):

					dataframe = pd.DataFrame(self.mem_data_block)
					dataframe.columns = self.element_of_Y + self.elements_of_X

					## Reinitialize mem_data_block
					self.mem_data_block = np.empty(len(self.elements_of_X) + len(self.element_of_Y))
					self.is_init = True
					## Multiple regression model building
					self.thread_list.append( executor.submit(self.regression_part, dataframe) )
				
				# else
				elif valid_flag == 1:
					self.get_logger().warn('Executing CLF skipped.. (self.mem_data_block: {})'.format(len(self.mem_data_block)))

		else:
			self.get_logger().warn('Invalid data received.')
			#raise ValueError
		
		end = time.time()
		self.get_logger().info('mem_data_block: {:2d}, valid_flag: {}, raptime: {:.4f}'.format(
			len(self.mem_data_block), valid_flag, end - start)
		)


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
