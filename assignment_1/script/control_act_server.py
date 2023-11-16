#! /usr/bin/env python

import rospy
import actionlib
import math
import assignment_1.msg as a1_msgs
from assignment_1.msg import ControlAction, RobotVision
from geometry_msgs.msg import Twist

id_marker = 0
vision_id = 0
camera_center = []
marker_center = []
marker_top_right = []
marker_top_left = []
marker_bottom_left = []
marker_bottom_right = []

class RobotControllerServer(object):
    """
    Action server controller for
     - looking for a target marker id
     - moving close to it
    """
    def __init__(self):
        # create feedback message
        self._feedback = a1_msgs.RobotControllerFeedback()
        # create result message
        self._result = a1_msgs.RobotControllerResult()
        
        # target marker id
        self._target_id = None
        
        # keep on getting close till the target marker side size
        # is lower than the threshold
        self._marker_side_th = None
        # tollerance error on marker side
        self._marker_side_tollerance = 1

        # camera data
        self._camera_info = RobotVision()
        
        # subscriber to camera topic
        self._vision_sub = rospy.Subscriber(
                                'info_vision',
                                RobotVision,
                                self.vision_callback
                            )
        
        # get process rate
        self._rate = rospy.get_param("process_rate")

        # publisher of the velocity reference control to the robot
        self._ctr_cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        # create action server
        self._act_server = actionlib.SimpleActionServer(
                                "robotController",
                                a1_msgs.RobotControllerAction,
                                execute_cb=self.act_server_callback,
                                auto_start=False
                            )
        
        # start action server
        self._act_server.start()

    def act_server_callback(self, goal: a1_msgs.RobotControllerGoal):
        """
        Handles the goal requests
         - case of search, it looks for the target marker id
         - case of get close, it navigates toward the target marker

        :param: goal: goal request from the client
        :type goal: assignment.msg.RobotControllerGoal

        :retur: None
        """

        if goal.action == ControlAction.SEARCH_ID:
            # get target marker id to look for
            self._target_id = goal.id
            success = self.search_marker(self._target_id)
            self._act_server.set_succeeded(result = a1_msgs.RobotControllerResult(success))

        elif goal.action == ControlAction.GET_CLOSE:
            self._marker_side_th = goal.marker_side_th
            # rospy.loginfo("get command get close - self._target_id: %d" % self._target_id)
            success = self.get_close_to_marker(self._marker_side_th)
            self._act_server.set_succeeded(result = a1_msgs.RobotControllerResult(success))
        

    def search_marker(self, id: int):
        """
        Looks for the target marker id rotating the robot

        :param: id: id of the target marker to look for
        :type id: int

        :return: None
        """

        r = rospy.Rate(self._rate)

        # velocity reference control
        velocity = Twist()
        velocity.angular.z = 0
        velocity.linear.x = 0

        # send velocity command
        self._ctr_cmd_vel_pub.publish(velocity)
        
        while(id != self._camera_info.id):

            rospy.loginfo("control - looking for marker id: %d" % id)

            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("control_act_server - Searching goal premented")
                # preempt current goal
                self._act_server.set_preempted()
                return False
            
            # send feedback on current id found by the camera
            if self._camera_info.id != None:
                self._feedback.id = self._camera_info.id
                self._act_server.publish_feedback(self._feedback)

            # keep on looking
            velocity.angular.z = -0.5
            self._ctr_cmd_vel_pub.publish(velocity)

            r.sleep()
        
        # stop looking for target marker id
        velocity.angular.z = 0
        self._ctr_cmd_vel_pub.publish(velocity)
                
        return True

    def get_close_to_marker(self, marker_side_th: float):
        """
        Gets close to the target marker up to a threshold.
        It keeps the target marker in the middel of the camera vision

        :param marker_side_th: threshold to stop the approach to the target marker
        :type marker_side_th: float

        :return: Bool: True, if the goal is succeeded; False if the goal fails
        """
        
        # control rate
        r = rospy.Rate(self._rate)

        # proportional gains
        linear_gain = 0.003
        angular_gain = 0.003

        # velocity reference
        velocity = Twist()

        # check just vertical side
        x_cord = self._camera_info.marker_top_right[0] - self._camera_info.marker_bottom_right[0]
        y_cord = self._camera_info.marker_top_left[1] - self._camera_info.marker_bottom_left[1]
            
        marker_side = math.sqrt(math.pow(x_cord,2) + math.pow(y_cord,2))
        
        # send feedback on the current marker size
        self._feedback.marker_side = marker_side
        self._act_server.publish_feedback(self._feedback)

        while(abs(marker_side - marker_side_th) > self._marker_side_tollerance):

            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("contro_act_server - Getting close to target marker preempted")
                # preempt current goal
                self._act_server.set_preempted()
                return False
            
            x_cord = self._camera_info.marker_top_right[0] - self._camera_info.marker_bottom_right[0]
            y_cord = self._camera_info.marker_top_left[1] - self._camera_info.marker_bottom_left[1]
            
            # consider just vertical side, since horizontal one could be seen sideways, thus,
            # in some case, could never reach the threshold
            marker_side = math.sqrt(math.pow(x_cord,2) + math.pow(y_cord,2))
            
            # send feedback on the current marker size
            self._feedback.marker_side = marker_side
            self._act_server.publish_feedback(self._feedback)

            angular_error = self._camera_info.camera_center[0] - self._camera_info.marker_center[0]
            linear_error = marker_side_th - marker_side
            
            velocity.linear.x = linear_gain * linear_error
            velocity.angular.z = angular_gain * angular_error
                
            self._ctr_cmd_vel_pub.publish(velocity)

            rospy.loginfo("control - marker side size error: %f" % linear_error)
            rospy.loginfo("control - camera center error: %f" % angular_error)

            # cycle timing
            r.sleep()
        
        # target marker reached
        velocity.linear.x = 0
        velocity.angular.z = 0
        self._ctr_cmd_vel_pub.publish(velocity)

        return True


    def vision_callback(self, vision_msg: RobotVision):
        """
        Stores the camera data from the camera topic /invo_vision

        :param vision_msg: camera data
        :type vision_msg: RobotVision

        :return: None
        """

        # filter out all messages not related to the target marker
        if(vision_msg.id == self._target_id):
            # rospy.loginfo("camera id == target id")
            self._camera_info.id = vision_msg.id
            self._camera_info.camera_center = vision_msg.camera_center
            self._camera_info.marker_center = vision_msg.marker_center
            self._camera_info.marker_top_right = vision_msg.marker_top_right
            self._camera_info.marker_top_left = vision_msg.marker_top_left
            self._camera_info.marker_bottom_left = vision_msg.marker_bottom_left
            self._camera_info.marker_bottom_right = vision_msg.marker_bottom_right


def main():
    """
    Main function of the node
      - initialize the node
      - creates action server object for ``robotController``
      - creates subscriber for topic ``/info_vision`` for getting camera data

    :return: None
    """
    rospy.init_node("robot_controller_action_server")
        
    ctr_act_server = RobotControllerServer()

    rospy.loginfo("robot_controller_action_server initialized")

    rospy.spin()


if __name__ == "__main__":
    main()

