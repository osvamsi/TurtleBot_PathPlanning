#!/usr/bin/env python

from search.srv import *
import rospy
from gen_maze import *
import sys
import argparse
import time

mazeInfo = None
parser = argparse.ArgumentParser()
parser.add_argument('-d', help='for providing dimension of the grid', metavar='5', action='store', dest='grid_dimension', default=5, type=int)
parser.add_argument('-n', help='for providing no. of obstacles to be added in the grid', metavar='15', action='store', dest='n_obstacles', default=15, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)

def manhattanDistance(x1, y1, x2, y2):
	"""
	This function returns manhattan distance between two points.
	"""
	return abs(x1-x2) + abs(y1-y2)

def check_is_edge(edge, valueFlag):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global mazeInfo
	invalid_edges = mazeInfo[1]
	if valueFlag == "changedValuesLater":
		if edge[2] < mazeInfo[0][0] or edge[2] > mazeInfo[0][1]*0.5 or edge[3] < mazeInfo[0][0] or edge[3] > mazeInfo[0][1]*0.5:
			return False
	elif valueFlag == "changedValuesBefore":
		if edge[0] < mazeInfo[0][0] or edge[0] > mazeInfo[0][1]*0.5 or edge[1] < mazeInfo[0][0] or edge[1] > mazeInfo[0][1]*0.5:
			return False

	if edge in invalid_edges:
		return False
	else:
		return True

def handle_get_successor(req):
	"""
		This function returns successor of a given state. 
				
		parameters:	x_cord - current x-cordinate of turtlebot           output:   x_cord - new x-cordinate of turtlebot
				    y_cord - current y-cordinate of turtlebot					  y_cord - new y-cordinate of turtlebot
				    direction - current orientation								  direction - new orientation
				    action - current action										  g_cost - Manhatan distance from initial state to new state
															      				  hurestic_value - Manhatan distance from goal state to new state
	"""
	global mazeInfo
	directionList = ["NORTH", "EAST","SOUTH","WEST"]
	x_cord, y_cord, direction, action = req.x, req.y, req.direction, req.action

	#Checking requested action and making changes in states
	if action == 'TurnCW':
		index = directionList.index(req.direction)
		direction = directionList[(index+1)%4]
		g_cost = 2

	elif action == 'TurnCCW':
		index = directionList.index(req.direction)
		direction = directionList[(index-1)%4]
		g_cost = 2

	elif action == 'MoveF':
		if direction == "NORTH":
			y_cord += 0.5
		elif direction == "EAST":
			x_cord += 0.5
		elif direction == "SOUTH":
			y_cord -= 0.5
		elif direction == "WEST":
			x_cord -= 0.5
		g_cost = 1

	elif action == 'MoveB':
		if direction == "NORTH":
			y_cord -= 0.5
		elif direction == "EAST":
			x_cord -= 0.5
		elif direction == "SOUTH":
			y_cord += 0.5
		elif direction == "WEST":
			x_cord += 0.5
		g_cost = 3
	
	if req.x <= x_cord and req.y <= y_cord:
		isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord), "changedValuesLater")
	else:
		isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y), "changedValuesBefore")

	if not isValidEdge:
		return GetSuccessorResponse(-1, -1, direction, -1)

	return GetSuccessorResponse(x_cord, y_cord, direction, g_cost)
  

def handle_get_initial_state(req):
	"""
	This function will return initial state of turtlebot3.
	"""
	global mazeInfo

	initial_state = mazeInfo[0]
	return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2])


def handle_is_goal_state(req):
	"""
    This function will return True if turtlebot3 is at goal state otherwise it will return False.
	"""
	global mazeInfo

	goal_state = mazeInfo[0][1]*0.5

	if req.x == req.y and req.x == goal_state:
		return IsGoalStateResponse(1)

	return IsGoalStateResponse(0)

def handle_get_goal_state(req):
	global mazeInfo
	goal_state = mazeInfo[0][1]*0.5
	return GetGoalStateResponse(goal_state,goal_state)

def server():
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('get_initial_state', GetInitialState, handle_get_initial_state)
    rospy.Service('is_goal_state', IsGoalState, handle_is_goal_state)
    rospy.Service('get_goal_state',GetGoalState,handle_get_goal_state)
    print "Ready!"
    rospy.spin()

if __name__ == "__main__":
    args = parser.parse_args()
    possible_n_obstacles =  args.grid_dimension*(args.grid_dimension + 1)*2
    if args.n_obstacles > possible_n_obstacles:
    	print('Maximum no. of obstacles that could be added to the grid is {} but provided value is {}'.format(possible_n_obstacles, args.n_obstacles))
    	exit()
    mazeInfo = generate_blocked_edges(args.grid_dimension, args.n_obstacles, args.seed)
    server()