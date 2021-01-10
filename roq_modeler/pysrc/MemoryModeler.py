# =============================
# MemoryModeler.py
# @discription	Modeling of acquired data and sending/receiving via ROS2
# @Author	Koki Nagahama (@hamstick)
# =============================

## Regression Kit
import numpy as np
import pandas as pd
import sklearn
from sklearn import linear_model
from concurrent.futures import ThreadPoolExecutor

## ROS2
import rclpy
from rclpy.node import Node
from ******.msg import ******	## Subscribe
from ******.msg import ******	## Publish

# Debug Utilities
import traceback

class MemoryModeler(Node):
	# Node & Topic Name
	NODENAME = 'mem_modeler'
	r_INTERVAL = 90

	## single robot version
	self.elements_of_X = \
		['buffer_szMB', \
		'cache_szMB', \
		'heap_bringup_szMB', 'heap_teleop_szMB', \
		'stack_bringup_szMB', 'stack_teleop_szMB']
	self.element_of_Y = ['system_mem_per']

	def __init__(self):
		super().__init__(self.NODENAME)
		self.get_logger().info('{} initializing...'.format(self.NODENAME))

		## Create ROS2 instance
		self.pub = self.create_publisher(******, "model_params")
		self.time_period = 1.00
		self.timer = self.create_timer(self.time_period, self.modeler_pub_callback)

		self.sub = self.create_subscription(ModelParams, "model_params", self.modeler_sub_callback)

		## Create REG instance
		self.data_array = np.array()
	
	def __del__(self):
		self.get_logger().info('{} done.'.format(self.NODENAME))

	## Multi Linear Regression @Multi-Threading
	def regression_part(self, dataflame):
		clf = linear_model.LinearRegression()
		clf.fit(dataflame[self.elements_of_X], dataflame[self.element_of_Y])

	## callback function when publish message
	def modeler_pub_callback(self):
		## Message setting
		msg = MemProc()
		msg = 
	
	## callback function when subscribe message
	"""
		Discription:
			Checking and storing elements arriving by Subscribe, data frame conversion
	"""
	def modeler_sub_callback(self, message):
		mem_data_arrival = np.array(
			[
				message.******, message.******, message.******, message.******, message.******, message.******
			]
		)
		
		if not(len(mem_data_arrival) == len(self.elements_of_X) + len(selt.element_of_Y)):
			raise NANTOKAException
		else:
			try:
				mem_data_block = np.vstack( (mem_data_block, mem_data_arrival) )
			except NameError:
				mem_data_block = np.empty(len(self.elements_of_X) + len(self.element_of_Y))
				mem_data_block = np.vstack( (mem_data_block, mem_data_arrival) )
				mem_data_block = np.delete(a, 0, axis = 0)
			finally:
				if len(mem_data_block) >= 90:
					dataflame = pd.DataFrame(mem_data_block)
					dataflame.columns = self.element_of_Y + self.elements_of_X
					predict_params = regression_part(dataflame)


def main():
	rclpy.init(args = args)
	node = MemoryModeler()
	try:
		rclpy.spin(node)
	except NANTOKAException:
		print('Exception raised..')
		traceback.print_exc()
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
