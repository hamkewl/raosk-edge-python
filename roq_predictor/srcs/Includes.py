# =============================
# Includes.py
# @discription	Class file for Predictor.py
# @Author				Koki Nagahama (@hamstick)
# =============================

# from Meminfo
class MemProc:
	vgid = -1
	childs = []
	buffer_sz = 0.
	cache_sz = 0.
	heap_sz = 0.
	stack_sz = 0.

	def __init__(self):
		pass

#from Netinfo
class NwProc:
	n_send = 0
	n_receive = 0

	def __init__(self):
		pass

# from Predictor.Mem
class MemParams:
	vgid = -1
	childs = []
	p_buffer = 0.
	p_cache = 0.
	p_heap = 0.
	p_stack = 0.
	p_intercept = 0.

	def __init__(self):
		pass

# from Predictor.Network
class NwParams:
	p_ave_send = 0.
	p_ave_receive = 0.

	def __init__(self):
		pass
