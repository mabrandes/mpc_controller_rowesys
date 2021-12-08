#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from reference_state_generator.msg import states_mpc
from visualization_msgs.msg import Marker

import math
import numpy as np
from numpy import linalg  


import math 


class Generator():

	def __init__(self):

		self.ref_time_ahead = 1.5

		self.test = 0
		self.angle_ref = 0
		self.angle_rob = 0
		self.position_ref = np.array([0,0])
		self.position_rob = np.array([0,0])
		self.dir_vec_ref = np.array([0,0])
		self.dir_vec_rob = np.array([0,0])

		self.init = 0
		self.offset_position = np.array([0,0])
		self.offset_orient = 0
		self.orientation_zeroOf = np.array([0, 0, 0, 0])

		self.dt = 0.0

		#otherwise first dt will be huge
		self.dt_p = rospy.get_time()
		self.it = 0

		self.distance = 0
		self.angle_phi = 0
		self.angle_alpha = 0
		self.angle_beta = 0

		self.test=0
		self.marker = Marker()
		

		self.sub_se = rospy.Subscriber ('/ov_msckf/poseimu', PoseWithCovarianceStamped, self.states_ref_callback, queue_size=100)
		self.sub_se = rospy.Subscriber ('/gazebo/model_states', ModelStates, self.states_rob_callback, queue_size=100)


	def states_ref_callback(self,msg):

		orientation_q = msg.pose.pose.orientation
		arr_orient = [orientation_q.x, orientation_q.y,orientation_q.z,orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(arr_orient)

		pos = msg.pose.pose.position

		#get initial offset
		if self.init == 0:
			self.offset_position = np.array([pos.x, pos.y])
			self.offset_orient = yaw
			self.init += 1

		#seconds in float type
		self.dt=rospy.get_time()-self.dt_p

		#generate ref pose
		if self.dt >= self.ref_time_ahead:
			print("hurraaaaa: ", self.dt, self.it)
			self.it += 1
			self.dt_p = rospy.get_time()

			self.angle_ref = yaw - self.offset_orient
			self.orientation_zeroOf = quaternion_from_euler(roll, pitch, self.angle_ref)
			self.position_ref=np.array([pos.x, pos.y]) - self.offset_position

			l=1
			self.dir_vec_ref=np.array([l*np.cos(self.angle_ref),l*np.sin(self.angle_ref)])

			#test mpc_state function
			# self.d_angle = (self.angle_ref-self.angle_rob)*180/math.pi
			#self.distance = np.linalg.norm(self.position_ref-self.position_rob)
			#print("test :", self.test)

	
	
	def states_rob_callback(self,msg):

		orientation_q = msg.pose[1].orientation
		arr_orient = [orientation_q.x, orientation_q.y,orientation_q.z,orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(arr_orient)

		self.angle_rob=yaw

		l=1
		self.dir_vec_rob=np.array([l*np.cos(yaw),l*np.sin(yaw)])

		pos=msg.pose[1].position
		self.position_rob=np.array([pos.x, pos.y])
	

		self.mpc_states()

	
	def mpc_states(self):

		if self.it >= 1 :
			dp = self.position_ref-self.position_rob
			self.angle_phi = self.angle_ref-self.angle_rob 		#*180/math.pi
			self.angle_alpha = -self.angle_phi+math.atan2(dp[1],dp[0])
			self.angle_beta = -self.angle_phi-self.angle_alpha
			self.distance = np.linalg.norm(dp)
			

	def path_line(self, frame_id):
		
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


	def create_line_marker(self, frame_id):

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
        # print "Pt 1 ", pt1
        # print "Pt 2 ", pt2
		points.append(pt1)
		points.append(pt2)
		mark.points = points

		return mark
	

def main():


	rospy.init_node('ref_state_generator')

	#initialize all variables in class
	gen=Generator()


	#sub_simu = rospy.Subscriber ('/gazebo/model_states', ModelStates, get_rotation_states)
	pub_ref_pose = rospy.Publisher('/ref_pose', PoseStamped, queue_size=10)
	pub_dist_angle = rospy.Publisher('/mpc_states', states_mpc, queue_size=10)
	pub_e = rospy.Publisher('/euler_yaw', Float64, queue_size=10)
	pub_marker = rospy.Publisher("dist_visualization", Marker)
	pub_path = rospy.Publisher("path_visualization", Marker, queue_size=10)

	#initialize
	ref_pose=PoseStamped()
	states=states_mpc()
	test=Float64()


	rate = rospy.Rate(50) # 10hz
	

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
		ref_pose.pose.orientation.x = gen.orientation_zeroOf[0]
		ref_pose.pose.orientation.y = gen.orientation_zeroOf[1]
		ref_pose.pose.orientation.z = gen.orientation_zeroOf[2]
		ref_pose.pose.orientation.w = gen.orientation_zeroOf[3]

		
		
		pub_ref_pose.publish(ref_pose)
		#rospy.loginfo(ref_pose)

		pub_dist_angle.publish(states)
		#rospy.loginfo(states)

		test.data=rospy.get_time()
		pub_e.publish(test)
		#rospy.loginfo(test)

		marker1=gen.create_line_marker("global")
		pub_marker.publish(marker1)

		marker2=gen.path_line("global")
		pub_path.publish(marker2)
		

		rate.sleep()




if __name__ == '__main__':
    main()
