#! /usr/bin/python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import numpy as np

class RandomWalk:

	def __init__(self):
		rospy.init_node("random_walk")
		self.publisher = rospy.Publisher("/actions",String,queue_size = 10)
		self.subscriber = rospy.Subscriber("/status",String,self.callback)
		self.init_state = problem.get_initial_state()
		self.current_state = self.init_state
		self.last_action = None
		rospy.Rate(1).sleep()
		print "Running"
		self.next_action()
		rospy.spin()

	def random_walk(self):	
		action_set = problem.get_actions()
		next_possible_states = list()
		for action in action_set:
			next_possible_state,cost = problem.get_successor(self.current_state,action)
			if next_possible_state.x != -1:
				next_possible_states.append((next_possible_state,action))
		rand_number = np.random.choice(range(len(next_possible_states)))
		selected_state, action = next_possible_states[rand_number]
		self.last_action = action
		print(selected_state,action)
		return selected_state,action

	def next_action(self):
		next_state,action = self.random_walk()
		self.current_state = next_state
		action_str = String(data = action)
		self.publisher.publish(action_str)

	def callback(self,data):
		if data.data == "next":
			self.next_action()


if __name__ == "__main__":
	random_walker = RandomWalk()
