#!/usr/bin/env python


import numpy as np
import numpy.random as npr

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from rowesys_navigation_msgs.msg import AutonomousMode, RowPosition
from rowesys_base_msgs.msg import MotorInputs
from reference_state_generator.msg import States_mpc




class PID():
	
	def __init__(self):
		
		self.start_pid = True

		self.alpha = 0
		self.beta = 0
		self.rho = 0

		self.k_beta=-1
		self.k_rho=0.3
		self.k_alpha=-5/3*self.k_beta+2/math.pi*self.k_rho+0.1
		#self.k_alpha=8

		self.timestamp = rospy.get_time()
		self.dt=0

		self.vel=0
		self.omega=0

		self.line_subscriber = rospy.Subscriber('/mpc_states', States_mpc, self.callback, queue_size=1)


	def callback(self, data):

		self.rho= data.dist
		self.alpha= data.angle_alpha
		self.beta = data.angle_beta

		self.vel= self.k_rho*self.rho
		self.omega = self.k_alpha*self.alpha+self.k_beta*self.beta

		self.dt=rospy.get_time()-self.timestamp
		self.timestamp=rospy.get_time()
		print(self.dt)


def main():
	
	rospy.init_node('rowesys_simple_pid_node')
	
	pid = PID()

    # Create a rate
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if pid.start_pid and not pid.dt == 0:
            
            # publish robot twist

			robot_twist_pub = rospy.Publisher('/rowesys/rowesys_ackermann_steering_controller/cmd_vel', Twist, queue_size=100)
			pid_pub = rospy.Publisher('/mpc_runtime', Float64, queue_size=100)

			twist_msg = Twist()
			pid_msg = Float64()

			#sum_dt = (rospy.get_rostime()).to_sec() - pid.timestamp.to_sec()


			#index = int(sum_dt/pid.dt)
			pid_msg.data=pid.dt
			pid_pub.publish(pid_msg)
            #print("index: ", index)


			twist_msg.linear.x = pid.vel
			twist_msg.linear.y = 0
			twist_msg.angular.z = pid.omega

			robot_twist_pub.publish(twist_msg)

		rate.sleep()


if __name__ == '__main__':
	main()
