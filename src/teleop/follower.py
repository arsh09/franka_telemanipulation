
import ctypes
import numpy as np
from ClassFile import RobotState

robot_state = RobotState()
# robot_state.q = (ctypes.c_double * 7)
# robot_state.q_d = np.array([1, 2, 3,4,5,6,7], dtype=np.float64)
# robot_state.q = (ctypes.c_double * 7)(*q)

# For RobotState q i.e joint position
print(robot_state.q)
robot_state.q[:] = range(1,8)
arr = [1,2,3,4,5,6,9]
robot_state.q[:] = arr
print([item for item in robot_state.q])

RobotState().q[:] = range(1,8)
print([item for item in RobotState().q])

###### TODO ###

# -----libfranka--------
# RoobotState class (robot_state.h)
#  	Duration class
# 	errors class

# Robot class  (robot.h)
# 	command_types.h
# 	control_types.h
# 	lowpass_filter.h



# -----Arshad code -------
# network_server.h 	
# examples_common.h 
# commons.h
# messages.h

################

_master_state = RobotState()	# TODO
_slave_state = RobotState()		# TODO
robot = Robot()					# TODO

server = NetworkServer()		# TODO

libname = os.path.abspath(os.path.join(os.path.dirname(__file__), "../build/follower"))
libc = ctypes.CDLL(libname)

# PrintNumber = wrap_function(libc, 'PrintNumber', None, [ctypes.POINTER(Addition)])
class Follower(ctypes.Structure):
	_fields_ = [("is_received", ctypes.c_bool),
	("follower_ip", ctypes.POINTER(ctypes.c_char)),
	("server_port", ctypes.POINTER(ctypes.c_char)),
	("server_ip", ctypes.POINTER(ctypes.c_char))
	]

	def __init__(self):
		self.InitializeRobot()

	def wrap_function(lib, funcname, restype, argtypes):
		"""Simplify wrapping ctypes functions"""
		func = lib.__getattr__(funcname)
		func.restype = restype
		func.argtypes = argtypes
		return func

	def InitializeRobot(self):
		# Not completed
		InitializeRobot = wrap_function(libc, 'InitializeRobot', None, None)
		pass

	def GoHome(self):
		pass

	def Control(self):
		pass

	def Read(self):
		pass

fol = Follower()
# fol.



