#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import Queue as queue
from Queue import PriorityQueue
import time

rospy.init_node("search_algorithm")

publisher = rospy.Publisher("/actions",String,queue_size =10)

parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)




def bfs():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    print "Running"
    explored = []
    queue_search = queue.Queue()
    queue_search.put((init_state,[]))
    flag = 0
    while True :
    	print("inside while")
	#path = queue.pop(0)
	#node = path[-1]
	#current_actions = list_of_actions.pop(0)	
	(curr_node,curr_action_list) = queue_search.get()
	explored.append(curr_node)
	
	
	for action in possible_actions:
	
		(next_possible_state, cost) = problem.get_successor(curr_node,action)
		  
		if next_possible_state not in explored and queue_search: 
			t_action_list =[]
			for temp_action in curr_action_list :
				t_action_list.append(temp_action)
			t_action_list.append(action)
			queue_search.put((next_possible_state,t_action_list))
			if next_possible_state == goal_state :
				print("goal reached")
				flag = 1
				end = time.time()
    				print(end - start)	
				action_list = t_action_list
				break
	if flag == 1: 
		break
	end = time.time()
	print (end-start)
				
    print "outside while"			
    			
				
    return action_list

def ucs():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []
    print "Running"
    explored = []
    queue_search = PriorityQueue()
    queue_search.put((0,(init_state,[])))
    flag = 0
    curr_cost = 0
    while True :
    	print("inside while")
	#path = queue.pop(0)
	#node = path[-1]
	#current_actions = list_of_actions.pop(0)	
	(curr_cost,(curr_node,curr_action_list)) = queue_search.get()
	explored.append(curr_node)
	
	
	for action in possible_actions:
	
		(next_possible_state, cost) = problem.get_successor(curr_node,action)
		  
		if next_possible_state.x != -1 and next_possible_state not in explored and queue_search: 
			t_action_list =[]
			child_cost = curr_cost + cost 
			for temp_action in curr_action_list :
				t_action_list.append(temp_action)
			t_action_list.append(action)
			queue_search.put((child_cost,(next_possible_state,t_action_list)))
			if next_possible_state == goal_state :
				flag = 1
				print("goal reached")
				end = time.time()
    				print(end - start)	
				action_list = t_action_list
				break
	if flag == 1: 
		break

	end = time.time()
	print (end-start)			
    print "outside while"
    if flag == 0:
    	return "no path found"
    else :
    	return action_list
def manhattan(state1,state2):
    	return abs(state2.x - state1.x) + abs(state2.y - state1.y)
def new_heuristic(state):
	return abs(1/state)
def gbfs():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []
    print "Running"
    explored = []
    queue_search = PriorityQueue()
    curr_cost = manhattan(init_state,goal_state)
    queue_search.put((curr_cost,(init_state,[])))
    flag = 0
    
    while True :
    	#print("inside while")
	#path = queue.pop(0)
	#node = path[-1]
	#current_actions = list_of_actions.pop(0)	
	(curr_cost,(curr_node,curr_action_list)) = queue_search.get()
	explored.append(curr_node)
	
	
	for action in possible_actions:
	
		(next_possible_state, cost) = problem.get_successor(curr_node,action)
		  
		if next_possible_state.x != -1 and next_possible_state not in explored and queue_search: 
			t_action_list =[]
			child_cost = manhattan(curr_node,goal_state) 
			print(child_cost)
			for temp_action in curr_action_list :
				t_action_list.append(temp_action)
			t_action_list.append(action)
			queue_search.put((child_cost,(next_possible_state,t_action_list)))
			if next_possible_state == goal_state :
				flag = 1
				action_list = t_action_list
				print("goal reached")
				end = time.time()
    				print(end - start)	
				break
	if flag == 1: 
		break

				
    print "outside while"			
    end = time.time()
    print(end-start)			
    
    if flag == 0:
    	return "no path found"
    else :
    	return action_list
    
def astar():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []
    print "Running"
    explored = []
    queue_search = PriorityQueue()
    curr_cost = manhattan(init_state,goal_state)
    queue_search.put((curr_cost,(init_state,[])))
    flag = 0
    
    while True :
    	#print("inside while")
	#path = queue.pop(0)
	#node = path[-1]
	#current_actions = list_of_actions.pop(0)	
	(curr_cost,(curr_node,curr_action_list)) = queue_search.get()
	explored.append(curr_node)
	
	
	for action in possible_actions:
	
		(next_possible_state, cost) = problem.get_successor(curr_node,action)
		  
		if next_possible_state.x != -1 and next_possible_state not in explored and queue_search: 
			t_action_list =[]
			child_cost = manhattan(curr_node,goal_state) + cost
			print(child_cost)
			for temp_action in curr_action_list :
				t_action_list.append(temp_action)
			t_action_list.append(action)
			queue_search.put((child_cost,(next_possible_state,t_action_list)))
			if next_possible_state == goal_state :
				flag = 1
				action_list = t_action_list
				print("goal reached")
				end = time.time()
    				print(end - start)	
				break
	if flag == 1: 
		break
    if flag == 0:
    	return "no path found"
    else :
    	return action_list
				
    print "outside while"			
    		

    if flag == 0:
    	return "no path found"
    else :
    	return action_list


   

 # to execute a plan action_list = <list of actions>, use:
def exec_action_list(actions):
    plan_str = '_'.join(action for action in actions)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    print algorithm
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    print "before receiving actions"
    actions = algorithm()
    print "before exec_action_list"
    exec_action_list(actions)



