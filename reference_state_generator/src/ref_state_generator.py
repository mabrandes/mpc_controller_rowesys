#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, compose_matrix, decompose_matrix, inverse_matrix
from reference_state_generator.msg import States_mpc
from visualization_msgs.msg import Marker

import math
import numpy as np
from numpy import linalg  


import math 


class Generator():

	def __init__(self):

		self.ref_time_ahead = 0.5

		self.test = 0
		self.angle_ref = [0, 0, 0]
		self.angle_rob = [0, 0, 0]
		self.position_ref = np.array([0,0,0])
		self.position_rob = np.array([0,0,0])
		self.offset_position = np.array([0,0,0])

		self.init = 0
		self.orientation_q_new = np.array([0, 0, 0, 0])
		self.orient_q_ref=[0, 0, 0, 0]
		self.orient_q_rob=[0, 0, 0, 0]

		self.yaw_extra=0.0
		self.dt = 0.0

		#otherwise first dt will be huge
		self.dt_p = rospy.get_time()
		self.it = 0

		self.distance = 0
		self.angle_phi = 0
		self.angle_alpha = 0
		self.angle_beta = 0

		self.marker = Marker()
		

		self.sub_se = rospy.Subscriber ('/ov_msckf/poseimu', PoseWithCovarianceStamped, self.states_ref_callback, queue_size=100)
		self.sub_twist = rospy.Subscriber ('/rowesys/rowesys_ackermann_steering_controller/odom', Odometry, self.twist, queue_size=100)
		self.sub_mod = rospy.Subscriber ('/gazebo/model_states', ModelStates, self.states_rob_callback, queue_size=100)

	def twist(self,msg):
		
		omega = msg.twist.twist.angular.z
		#y=0 for ackermann
		v_base = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

		dx=1.260+.186
		dy=(.846/2+.162)
		phi=math.atan2(dy,dx)
		d=math.sqrt(dx**2+dy**2)
		v_r=d*omega

		v=np.array([v_r*math.cos(phi),v_r*math.sin(phi)])
		v_tot=v+v_base
		self.yaw_extra=math.atan2(v_tot[1],v_tot[0])
		


	def states_ref_callback(self,msg):

		orientation_q = msg.pose.pose.orientation
		self.orient_q_ref = [orientation_q.x, orientation_q.y,orientation_q.z,orientation_q.w]
		(roll,pitch,yaw) = euler_from_quaternion(self.orient_q_ref)

		#angles in alphasense camera frame
		#angles_ref_camera = [0,0, yaw+self.yaw_extra]
		angles_ref_camera = [0,0, yaw]

		pos = msg.pose.pose.position
		position_ = np.array([pos.x, pos.y, 0])

		self.angle_ref, position = self.pose2trans_matrix(angles_ref_camera ,position_)
		#angle_ref is the same as angles_ref_camera

		#get initial offset
		if self.init == 0:
			#position_ = position of camera
			#position  = position of center of robot
			self.offset_position = position
			self.init += 1

		#seconds in float type
		self.dt=rospy.get_time()-self.dt_p

		#generate ref pose
		if self.dt >= self.ref_time_ahead:
			print("hurraaaaa: ", self.dt, self.it)
			self.it += 1
			self.dt_p = rospy.get_time()
			

			self.position_ref= position - self.offset_position
		
			#self.orientation_q_new = quaternion_from_euler(angles_ref_camera[0],angles_ref_camera[1],angles_ref_camera[2])
			self.orientation_q_new = quaternion_from_euler(self.angle_ref[0],self.angle_ref[1],self.angle_ref[2])

			

	def pose2trans_matrix(self,angles_camera, pos):

		#world,camera
		T_wc = compose_matrix(angles=angles_camera,translate=pos)

		x= 1.260+.186
		y= -(.846/2+.162)
		z= .404
		t=[x,y,z]
		#inverse of T_rc: only translations get negative of zero angles. 
		#output is numpy array
		T_cr = inverse_matrix(compose_matrix(angles=[0,0,0],translate=t))
		T_wr = T_wc.dot(T_cr)

		scale, shear, angles, trans, persp = decompose_matrix(T_wr)
		#print(angles)



		return angles,trans
	
	def states_rob_callback(self,msg):

		orientation_rob = msg.pose[1].orientation
		self.orient_q_rob = [orientation_rob.x, orientation_rob.y,orientation_rob.z,orientation_rob.w]
		(roll, pitch, yaw) = euler_from_quaternion(self.orient_q_rob)
		#print("orent_rob :", self.arr_orient_rob)

		pos=msg.pose[1].position

		self.position_rob=np.array([pos.x, pos.y,0])
		self.angle_rob=[0,0,yaw]

		self.mpc_states()

	
	def mpc_states(self):

		if self.it >= 1 :
			dp = self.position_ref-self.position_rob
			self.angle_phi = self.angle_ref[2]-self.angle_rob[2] 		#*180/math.pi
			self.angle_alpha = -self.angle_phi+math.atan2(dp[1],dp[0])
			self.angle_beta = -self.angle_phi-self.angle_alpha
			self.distance = np.linalg.norm(dp)
			

	def path_line_marker(self, frame_id):
		
		self.marker.header.frame_id = frame_id  # marker is referenced to base frame
		self.marker.header.stamp = rospy.Time.now()

		self.marker.type = self.marker.LINE_STRIP
		self.marker.action = self.marker.ADD
		self.marker.scale.x = 0.1  # define the size of marker
		self.marker.color.a = 1.0
		self.marker.color.g = 1.0

		p=Point()
		p.x=self.position_ref[0]
		p.y=self.position_ref[1]
		p.z= 0

		self.marker.points.append(p)

		return self.marker


	def distance_marker(self, frame_id):

        #create marker:person_marker, modify a red cylinder, last indefinitely
		mark = Marker()
		mark.header.frame_id = frame_id #tie marker visualization to laser it comes from
		mark.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
		mark.ns = "ref_to_rob"
		mark.id = 0
		mark.type = Marker.LINE_STRIP
		mark.action = 0
		mark.scale.x = 0.2 
		mark.color.a = 1.0
		mark.color.r = 1.0
		mark.color.g = 1.0
		mark.color.b = 1.0
		mark.text = "ref_to_rob"

		points = []
		p1=self.position_ref
		p2=self.position_rob
		pt1 = Point(p1[0], p1[1], 0)
		pt2 = Point(p2[0], p2[1], 0)

		points.append(pt1)
		points.append(pt2)
		mark.points = points

		return mark

	def pose_rob_marker(self, frame_id):

        #create marker:person_marker, modify a red cylinder, last indefinitely
		mark = Marker()
		mark.header.frame_id = frame_id #tie marker visualization to laser it comes from
		mark.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
		mark.ns = "orient_rob"
		mark.id = 0
		mark.type = Marker.ARROW
		#mark.type = Marker.SPHERE
		mark.action = 0
		mark.scale.x = 1
  		mark.scale.y = 0.1
  		mark.scale.z = 0.1
		# a= transparancy 1= no transparancy
		mark.color.a = 1.0
		mark.color.r = 1.0
		mark.color.g = 0.1
		mark.color.b = 0.1
 		mark.pose.position.x = self.position_rob[0]
		mark.pose.position.y = self.position_rob[1]
		mark.pose.position.z = self.position_rob[2]
		mark.pose.orientation.x = self.orient_q_rob[0]
		mark.pose.orientation.y = self.orient_q_rob[1]
		mark.pose.orientation.z = self.orient_q_rob[2]
		mark.pose.orientation.w = self.orient_q_rob[3]

		#print("rob :", self.arr_orient_rob)
		return mark
	
	def pose_ref_marker(self, frame_id):

        #create marker:person_marker, modify a red cylinder, last indefinitely
		mark = Marker()
		mark.header.frame_id = frame_id #tie marker visualization to laser it comes from
		mark.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
		mark.ns = "orient_ref"
		mark.id = 0
		mark.type = Marker.ARROW
		#mark.type = Marker.SPHERE
		mark.action = 0
		mark.scale.x = 1
  		mark.scale.y = 0.1
  		mark.scale.z = 0.1
		# a= transparancy 1= no transparancy
		mark.color.a = 1.0
		mark.color.r = 1.0
		mark.color.g = 0.1
		mark.color.b = 0.1
 		mark.pose.position.x = self.position_ref[0]
		mark.pose.position.y = self.position_ref[1]
		mark.pose.position.z = self.position_ref[2]
		mark.pose.orientation.x = self.orientation_q_new[0]
		mark.pose.orientation.y = self.orientation_q_new[1]
		mark.pose.orientation.z = self.orientation_q_new[2]
		mark.pose.orientation.w = self.orientation_q_new[3]

		#print("rob :", self.arr_orient_rob)
		return mark

def main():


	rospy.init_node('ref_state_generator')

	#initialize all variables in class
	gen=Generator()


	#sub_simu = rospy.Subscriber ('/gazebo/model_states', ModelStates, get_rotation_states)
	pub_ref_pose = rospy.Publisher('/ref_pose', PoseStamped, queue_size=10)
	pub_dist_angle = rospy.Publisher('/mpc_states', States_mpc, queue_size=10)
	pub_e = rospy.Publisher('/euler_yaw', Float64, queue_size=10)
	pub_dist= rospy.Publisher("dist_visualization", Marker)
	pub_path = rospy.Publisher("path_visualization", Marker, queue_size=10)
	pub_pose_rob = rospy.Publisher("pose_rob_visualization", Marker, queue_size=10)
	pub_pose_ref = rospy.Publisher("pose_ref_visualization", Marker, queue_size=10)

	#initialize
	ref_pose=PoseStamped()
	states=States_mpc()
	test=Float64()


	rate = rospy.Rate(20) # 10hz
	

	while not rospy.is_shutdown():


		states.header.stamp = rospy.Time.now()
		states.angle_phi = gen.angle_phi
		states.angle_alpha = gen.angle_alpha
		states.angle_beta = gen.angle_beta
		states.dist = gen.distance


		ref_pose.header.stamp = rospy.Time.now()
		ref_pose.header.frame_id = "global"
		ref_pose.pose.position.x = gen.position_ref[0]
		ref_pose.pose.position.y = gen.position_ref[1]
		ref_pose.pose.position.z = gen.position_ref[2]
		ref_pose.pose.orientation.x = gen.orientation_q_new[0]
		ref_pose.pose.orientation.y = gen.orientation_q_new[1]
		ref_pose.pose.orientation.z = gen.orientation_q_new[2]
		ref_pose.pose.orientation.w = gen.orientation_q_new[3]

		
		
		pub_ref_pose.publish(ref_pose)
		#rospy.loginfo(ref_pose)

		pub_dist_angle.publish(states)
		#rospy.loginfo(states)

		test.data=rospy.get_time()
		pub_e.publish(test)
		#rospy.loginfo(test)

		marker1=gen.distance_marker("global")
		pub_dist.publish(marker1)

		marker2=gen.path_line_marker("global")
		pub_path.publish(marker2)
		
		marker3=gen.pose_rob_marker("global")
		pub_pose_rob.publish(marker3)
		
		marker4=gen.pose_ref_marker("global")
		pub_pose_ref.publish(marker4)

		rate.sleep()




if __name__ == '__main__':
    main()
