#!/usr/bin/env python
import sys
import rospy
from search.srv import *

class State:

    def __init__(self,x,y,orientation):
        self.x  = x 
        self.y = y
        self.orientation = orientation

    def __eq__(self,other):
        if self.x == other.x and self.y == other.y and self.orientation == other.orientation:
            return True
        else:
            return False


def get_successor(state, action):
    """
    This function calls get_successor service with current state as input and receives the successor state as output. 
        
    parameters:  x_cord - current x-cordinate of turtlebot           return:   x_cord - new x-cordinate of turtlebot
                 y_cord - current y-cordinate of turtlebot                     y_cord - new y-cordinate of turtlebot
                 direction - current orientation                               direction - new orientation
                 action - current action                                       g_cost - Manhatan distance from initial state to new state
                                                                               hurestic_value - Manhatan distance from goal state to new state
    """
    rospy.wait_for_service('get_successor')
    try:
        get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
        response = get_successor(state.x,state.y,state.orientation, action)
        return State(response.x, response.y, response.direction), response.g_cost
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_initial_state():
    """
    This function calls get_initial_state service to recive the initial state of the turtlebot.

    return:  x_cord - initial x-cordinate of turtlebot           
             y_cord - initial y-cordinate of turtlebot
             direction - initial orientation
    """
    rospy.wait_for_service('get_initial_state')
    try:
        get_initial_state = rospy.ServiceProxy('get_initial_state', GetInitialState)
        response = get_initial_state()
        return State(response.x, response.y, response.direction)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def is_goal_state(state):
    """
    This function calls is_goal_state service to check if the current state is the goal state or not.

    parameters:  x_cord - current x-cordinate of turtlebot           return:   1 : if current state is the goal state
                 y_cord - current y-cordinate of turtlebot                     0 : if current state is not the goal state
    """
    rospy.wait_for_service('is_goal_state')
    try:
        is_goal_state_client = rospy.ServiceProxy('is_goal_state', IsGoalState)
        response = is_goal_state_client(state.x,state.y)
        return response.is_goal
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_goal_state():
    rospy.wait_for_service('get_goal_state')
    try:
        get_goal_state = rospy.ServiceProxy('get_goal_state',GetGoalState)
        response = get_goal_state()
        return State(response.x,response.y,"EAST")
    except rospy.ServiceException,e:
        print "Service call failed: %s"%e
def get_actions():
    return ["TurnCW","TurnCCW","MoveF","MoveB"]

def usage():
    return "%s [x y]"%sys.argv[0]
