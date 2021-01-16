# =============================
# NetworkModeler.py
# @discription	Modeling of acquired data and sending/receiving via ROS2
# @Author	Koki Nagahama (@hamstick)
# =============================

## Regression Kit
import time
import numpy as np
import pandas as pd
from concurrent.futures import ThreadPoolExecutor

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
	r_INTERVAL = 90

	## Instances
	thread_list = []
	#executor = ThreadPoolExecutor(max_workers = 8)

	## Model Parameters
	#MID = -1
	p_send = 0.00
	p_receive = 0.00
	p_intercept = 0.00
	
	## single robot version
	elements_of_X = [
		'n_send',
		'n_receive'
  ]
	#element_of_Y = ['n_load']
	#mem_data_block = np.empty(len(elements_of_X) + len(element_of_Y))
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

	## Create model
	def create_model(self, ndarray):
		


	"""
	def regression_part(self, dataframe):
		#self.get_logger().info('into REG')
		#clf = linear_model.LinearRegression()
		#clf.fit(dataframe[self.elements_of_X], dataframe[self.element_of_Y])

		## Setting message value
		predict_params = clf.coef_[0]
		self.get_logger().info('--> predict_params: {}'.format(
			np.round(np.append(clf.coef_[0], clf.intercept_), decimals = 4))
		)
		self.p_send = predict_params[0]
		self.p_receive = predict_params[1]
		#self.p_heap = predict_params[2]
		#self.p_stack = predict_params[3]
		self.p_intercept = clf.intercept_[0]
		self.get_logger().info('--> clf.score: {:.4f}'.format(
			clf.score(dataframe[self.elements_of_X], dataframe[self.element_of_Y[0]])
		))
	"""

	## callback function when publish message
	def modeler_pub_callback(self):
		## Message setting
		msg = NwParamsMsg()
		#msg.machine_id = self.MID


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
