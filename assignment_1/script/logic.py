#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray
import assignment_1.msg

# subscriber to get controller goal status
ctr_goal_status_sub = None

def ctr_goal_status_callback(ctr_goal_status: GoalStatusArray):
    
    global ctr_goal_status_sub

    # check if the list is empty
    if ctr_goal_status.status_list:
        # check goal status
        if (ctr_goal_status.status_list[0].status == actionlib.GoalStatus.ACTIVE):
            rospy.loginfo("goal_status.status_list[0].status: %d" % ctr_goal_status.status_list[0].status)

        if (ctr_goal_status.status_list[0].status == actionlib.GoalStatus.SUCCEEDED):
            
            ctr_goal_status_sub.unregister()
            rospy.loginfo("goal reached")

def main():

    global ctr_goal_status_sub

    # initialize node
    rospy.init_node("robot_logic")
    
    # client for controller
    ctr_client = actionlib.SimpleActionClient("robotController",assignment_1.msg.RobotControllerAction)

    # wait for the server controller to be started
    ctr_client.wait_for_server()

    # create subscriber for goal result
    ctr_goal_status_sub = rospy.Subscriber("/robotController/status", GoalStatusArray, ctr_goal_status_callback)

    # create goal
    ctr_goal = assignment_1.msg.RobotControllerGoal(action="find")

    # send the goal to the server
    ctr_client.send_goal(ctr_goal)

    rospy.loginfo("robot_logic node initialized")

    rospy.spin()

if __name__ == "__main__":
    main()
