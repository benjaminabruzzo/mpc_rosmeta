#!/usr/bin/env python
# Import required Python code.
import osqp
import numpy as np
import scipy as sp
import scipy.sparse as sparse
import yaml


# Ros Libs
import roslib
import rospy
from std_msgs.msg import Empty

class osqp_node():
	# Must have __init__(self) function for a class, similar to a C++ class constructor.
	def __init__(self):
		
		# output_topic = rospy.get_param('~output_topic')

		# subscribers and publishers
		trigger_topic = rospy.get_param('~trigger_topic')
		self.mpc_trigger_sub = rospy.Subscriber(trigger_topic,Empty, self.osqp_trigger)

		# model_yaml = rospy.get_param('~model_yaml')

		# Prediction horizon
		# N = 10
		N = rospy.get_param('~N')
		self.N=N
		dt = rospy.get_param('~dt')
		# dt = 0.25; # Assuming each increment is 0.25 seconds long

		Ad_yaml = rospy.get_param('~Ad')
		Ad_data = Ad_yaml['data']

		print(Ad_data)
		Ad=Ad_data
		for i in range(len(Ad_data)):
			# print("Ad_data[{}] = {}").format(i, Ad_data[i])
			if isinstance(Ad_data[i], str):
				Ad[i] = eval(Ad_data[i])
			# print("Ad[{}] = {}").format(i, Ad[i])

		test_mat = self.yaml_matrix2csc(Ad, Ad_yaml['rows'], Ad_yaml['rows'])
		print(test_mat)

		# Discrete time model of a 1D quadcopter
		self.Ad = sparse.csc_matrix([
		  [1.0,  dt, dt*dt/2],
		  [0.0, 1.0, dt],
		  [0.0, 0.0, 1.0]])
		self.Bd = sparse.csc_matrix([
		  [0.0],
		  [0.0],
		  [1.0]])
		[nx, nu] = self.Bd.shape
		self.nx = nx
		self.nu = nu

		# Objective function
		Q = sparse.diags([1.0, 1.0, 0.0])
		# Q = 1.0*sparse.eye(nx)
		QN = Q # Q at horizon N
		R = 0.1*sparse.eye(self.nu)

		# Constraints
		u0 = 0.0
		umin = np.array([-1.0]) - u0
		umax = np.array([ 1.0]) - u0
		xmin = np.array([-np.inf,-np.inf,-np.inf])
		xmax = np.array([ np.inf, np.inf, np.inf])

		# Initial and reference states
		self.x0 = np.zeros(3)
		xr = np.array([1.0, 0.0, 0.0])


		# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
		# - quadratic objective
		P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN, sparse.kron(sparse.eye(N), R)]).tocsc()
		# - linear objective
		# Stack all 
		q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr), np.zeros(N*self.nu)])
		# - linear dynamics
		Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
		Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
		Aeq = sparse.hstack([Ax, Bu])
		leq = np.hstack([-self.x0, np.zeros(N*nx)])
		ueq = leq
		# - input and state constraints
		Aineq = sparse.eye((N+1)*nx + N*self.nu)
		lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
		uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
		# - OSQP constraints
		A = sparse.vstack([Aeq, Aineq]).tocsc()
		self.l = np.hstack([leq, lineq])
		self.u = np.hstack([ueq, uineq])

		# Create an OSQP object
		self.osqp_solver = osqp.OSQP()

		# Setup workspace
		self.osqp_solver.setup(P, q, A, self.l, self.u, warm_start=True)

		while not rospy.is_shutdown():
			rospy.spin()

	def osqp_trigger(self,data):
		# Solve for current iteration
		self.res = self.osqp_solver.solve()
		# Check solver status
		if self.res.info.status != 'solved':
		    raise ValueError('OSQP did not solve the problem!')

		# Apply first control input to the plant
		self.ctrl = self.res.x[-self.N*self.nu:-(self.N-1)*self.nu]
		self.x0 = self.Ad.dot(self.x0) + self.Bd.dot(self.ctrl)

		print(self.ctrl)

		# Update initial state (Replace with callback at some point for actual vehicle state)
		self.l[:self.nx] = -self.x0
		self.u[:self.nx] = -self.x0
		self.osqp_solver.update(l=self.l, u=self.u)

	def list2csc(self,data, rows, cols):
		
		dt = 0.25
		csc_mat = sparse.csc_matrix([
		  [1.0,  dt, dt*dt/2],
		  [0.0, 1.0, dt],
		  [0.0, 0.0, 1.0]])
		return csc_mat



if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('osqp_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        c_rmp = osqp_node()
    except rospy.ROSInterruptException: pass


# rostopic pub -1 /mpc/osqp/trigger std_msgs/Empty