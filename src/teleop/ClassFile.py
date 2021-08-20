import ctypes
import os
import numpy

# libc = ctypes.CDLL('./trial.so')
libname = os.path.abspath(os.path.join(os.path.dirname(__file__), "/home/venkatesh/Lincoln/franka_panda_experiments/build/follower_franka"))
libc = ctypes.CDLL(libname)

class RobotState(ctypes.Structure):
	_fields_ = [('O_T_EE', ctypes.c_double * 16),	# Measured end effector pose in base frame.
	('O_T_EE_d', ctypes.c_double * 16),	# Last desired end effector pose of motion generation in base frame
	('F_T_EE', ctypes.c_double * 16),		# End effector frame pose in flange frame
	('F_T_NE', ctypes.c_double * 16),		# Nominal end effector frame pose in flange frame.
	('NE_T_EE', ctypes.c_double * 16),	# End effector frame pose in nominal end effector frame.
	('EE_T_K', ctypes.c_double * 16),		# Stiffness frame pose in end effector frame.
	('m_ee', ctypes.c_double),										# Configured mass of the end effector.
	('I_ee', ctypes.c_double * 9),		# Configured rotational inertia matrix of the end effector load with respect to center of mass.
	('F_x_Cee', ctypes.c_double * 3),	# Configured center of mass of the end effector load with respect to flange frame.
	('m_load', ctypes.c_double),									# Configured mass of the external load.
	('I_load', ctypes.c_double * 9),		# Configured rotational inertia matrix of the external load with respect to center of mass.
	('F_x_Cload', ctypes.c_double * 3),	# Configured center of mass of the external load with respect to flange frame.
	('m_total', ctypes.c_double),									# Sum of the mass of the end effector and the external load.
	('I_total', ctypes.c_double * 9),	# Combined rotational inertia matrix of the end effector load and the external load with respect to center of mass
	('F_x_Ctotal', ctypes.c_double * 3),	# Combined center of mass of the end effector load and the external load with respect to flange
	('elbow', ctypes.c_double * 2),		# Elbow configuration. The values of the array are: [0] Position of the 3rd joint in [rad]; [1] Sign of the 4th joint. Can be +1 or -1.
	('elbow_d', ctypes.c_double * 2),	# Desired elbow configuration. The values of the array are: [0] Position of the 3rd joint in [rad]; [1] Sign of the 4th joint. Can be +1 or -1.
	('elbow_c', ctypes.c_double * 2),	# Commanded elbow configuration. The values of the array are: [0] Position of the 3rd joint in [rad]; [1] Sign of the 4th joint. Can be +1 or -1.
	('delbow_c', ctypes.c_double * 2),	# Commanded elbow velocity. The values of the array are: [0] Position of the 3rd joint in [rad]; [1] Sign of the 4th joint. Can be +1 or -1.
	('ddelbow_c', ctypes.c_double * 2),	# Commanded elbow acceleration. The values of the array are: [0] Position of the 3rd joint in [rad]; [1] Sign of the 4th joint. Can be +1 or -1.
	
	('tau_J', ctypes.c_double * 7),	# Measured link-side joint torque sensor signals in Nm
	('tau_J_d', ctypes.c_double * 7),	# Desired link-side joint torque sensor signals without gravity in Nm
	('dtau_J', ctypes.c_double * 7),	# Derivative of measured link-side joint torque sensor signals in Nms
	('q', ctypes.c_double * 7),	# Measured joint position in rad
	('q_d', ctypes.c_double * 7),	# Desired joint position in rad
	('dq', ctypes.c_double * 7),	# Measured joint velocity in rad/s
	('dq_d', ctypes.c_double * 7),	# Desired joint velocity in rad/s
	('ddq_d', ctypes.c_double * 7),	# Desired joint acceleration in rad/s^2
	('joint_contact', ctypes.c_double * 7),	# Indicates which contact level is activated in which joint. After contact disappears, value turns to zero
	('cartesian_contact', ctypes.c_double * 6),	# Indicates which contact level is activated in which Cartesian dimension \f$(x,y,z,R,P,Y)\f$. After contact disappears, the value turns to zero.
	('joint_collision', ctypes.c_double * 7),	# Indicates which contact level is activated in which joint. After contact disappears, the value stays the same until a reset command is sent.
	('cartesian_collision', ctypes.c_double * 6),	# Indicates which contact level is activated in which Cartesian dimension \f$(x,y,z,R,P,Y)\f$., the value stays the same until a reset command is sent.
	('tau_ext_hat_filtered', ctypes.c_double * 7),	# External torque, filtered in Nm
	('O_F_ext_hat_K', ctypes.c_double * 6),	# Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame.
	('K_F_ext_hat_K', ctypes.c_double * 6),	# Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame
	('O_dP_EE_d', ctypes.c_double * 6),	# Desired end effector twist in base frame.
	('O_T_EE_c', ctypes.c_double * 16),	# Last commanded end effector pose of motion generation in base frame.
	('O_dP_EE_c', ctypes.c_double * 6),	# Last commanded end effector twist in base frame.
	('O_ddP_EE_c', ctypes.c_double * 6),	# Last commanded end effector acceleration in base frame.
	('theta', ctypes.c_double),	# Motor position in rad.
	('dtheta', ctypes.c_double),	# Motor velocity in rad/s.
	('control_command_success_rate', ctypes.c_double),	# Last commanded end effector acceleration in base frame.
	]

	# def __init__(self,q=None):
	# 	if q:
	# 		self.q = q
	# 	else:
	# 		q = (ctypes.c_double * 7)()
	# 		q[:] = range(1,8)
	# 		self.q = q


# read_loop = libc.read_loop()
# libc.read_loop.argtypes = [ctypes.c_bool]
# libc.read_loop.restype = ctypes.c_bool
# val = libc.read_loop(False)


# libc.control_loop.restype = ctypes.c_double * 16
# libc.control_loop.argtypes = [ctypes.POINTER(RobotState), ctypes.frank.Duration]
# arr_size = 7
# DoubleArr = ctypes.c_double * arr_size

