#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf

class moveTbot3:
	def __init__(self):
		rospy.init_node('move_turtle',anonymous = True)
		self.actions = String()
		self.pose = Pose()
		self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.callback)
		self.pose_subscriber = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.free = String(data = "next")
		print("Here")
		self.rate = rospy.Rate(30)
		rospy.spin()

	def callback(self,data):
		self.actions = data.data
		print self.actions
		self.rate.sleep()
		self.move()

	def pose_callback(self,data):
		self.pose = data.pose.pose

	def move(self):
		vel_msg = Twist()
		for action in self.actions.split("_"):
			init_pose = self.pose
			quat = (init_pose.orientation.x,init_pose.orientation.y,init_pose.orientation.z,init_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = euler[2]
			position = init_pose.position


			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0
		
			if action == "MoveF" or action == "MoveB":
				speed = 0.15
				if action == "MoveF":
					vel_msg.linear.x = speed
				else:
					vel_msg.linear.x = -speed
				t0 = rospy.Time.now().to_sec()
				distance = 0.5
				current_distance = 0
				while current_distance < distance:
					self.vel_pub.publish(vel_msg)
					t1 = rospy.Time().now().to_sec()
					current_distance = speed * (t1-t0)
				vel_msg.linear.x = 0
				vel_msg.angular.z = 0
				self.vel_pub.publish(vel_msg)
			elif action == "TurnCW" or action == "TurnCCW":
				speed = 0.15
				if action == "TurnCW":
					vel_msg.angular.z = -speed
					target_yaw = yaw - ( math.pi / 2.0)
					if target_yaw < -math.pi:
						target_yaw += (math.pi * 2)
				else:
					vel_msg.angular.z = speed
					target_yaw = yaw + ( math.pi / 2.0)
					if target_yaw >= (math.pi ):
						target_yaw -= (math.pi * 2 )
				quat = (self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w)
				euler = tf.transformations.euler_from_quaternion(quat)
				current_yaw = euler[2]
				while abs(current_yaw - target_yaw) > 0.1e-2:
					if action == "TurnCW":
						vel_msg.angular.z = -min(speed,min(speed*1.5,abs(current_yaw - target_yaw)/1.5))
					elif action == "TurnCCW":
						vel_msg.angular.z = min(speed,min(speed*1.5,abs(current_yaw - target_yaw)/1.5))
					self.vel_pub.publish(vel_msg)
					quat = (self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w)
					euler = tf.transformations.euler_from_quaternion(quat)
					current_yaw = euler[2]
				vel_msg.angular.z = 0
				self.vel_pub.publish(vel_msg)
			self.status_publisher.publish(self.free)

if __name__ == "__main__":
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass
