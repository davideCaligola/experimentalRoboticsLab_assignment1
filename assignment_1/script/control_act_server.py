#! /usr/bin/env python

from typing import Counter
import rospy
import actionlib

import assignment_1.msg

id = 12

class RobotControllerServer(object):
    # create feedback message
    _feedback = assignment_1.msg.RobotControllerFeedback()
    _result = assignment_1.msg.RobotControllerResult()

    def __init__(self):
        # id list
        rospy.loginfo("_feedback")
        rospy.loginfo(self._feedback)
        ids = []
        # create action server
        self._act_server = actionlib.SimpleActionServer("robotController", assignment_1.msg.RobotControllerAction,
                                                        execute_cb=self.act_server_callback, auto_start=False)
        # start action server
        self._act_server.start()

    def act_server_callback(self, goal: assignment_1.msg.RobotControllerGoal):
        global ids
        success = True

        id = 13

        r = rospy.Rate(1)

        rospy.loginfo("Executing goal %s" % goal)

        for counter in range(0,5):
            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("%s: premented" % goal)
                # preempt current goal
                self._act_server.set_preempted()
                success = False
                break
            
            # update and send feedback
            # by hypothesis there is always just one id
            if id != None:
                self._feedback.ids.append(id)
                self._act_server.publish_feedback(self._feedback)
            
            rospy.loginfo("counter: %d" % counter)

            r.sleep()
        
        if (success):
            self._result.ids.append(id)
            self._act_server.set_succeeded(self._result)

    
def main():
    rospy.init_node("robot_controller_action_server")
        
    rospy.loginfo("robot_controller_action_server initialized")

    controller_act_server = RobotControllerServer()

    rospy.spin()


if __name__ == "__main__":
    main()

