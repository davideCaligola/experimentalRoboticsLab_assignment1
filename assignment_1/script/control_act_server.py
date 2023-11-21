#! /usr/bin/env python

import rospy
import actionlib
import math
import assignment_1.msg as a1_msgs
from assignment_1.msg import RobotVision
from geometry_msgs.msg import Twist


class RobotCtrl_base(object):
    """
    Base class for Robot Controller. It provides:
     - common members,
     - publisher on /cmd_vel to control the robot,
     - subscriber callback to get data from the camera
    """

    def __init__(self):

        # target marker id
        self._target_id = None

        # get process rate
        self._rate = rospy.get_param("process_rate")

        # subscriber for gettinc camera info
        self._vision_sub = None

        # timout [s] for the first camera update
        self._camera_timeout = 2.0

        # camera data
        self._camera_info = RobotVision()

        # publisher of the velocity reference control to the robot
        self._ctr_cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        
    def vision_callback(self, vision_msg: RobotVision):
        """
        Stores the camera data from the camera topic /info_vision

        :param vision_msg: camera data
        :type vision_msg: RobotVision

        :return: None
        """

        # filter out all messages not related to the target marker
        self._camera_info.ids = vision_msg.ids
        if( vision_msg.ids and vision_msg.ids[0] == self._target_id):
            self._camera_info = vision_msg


class RobotCtrlServer_searchMarkerId(RobotCtrl_base):
    """
    Action server controller for searching a target marker id
    """
    def __init__(self):

        # initialize parent class
        super().__init__()

        # create feedback message
        self._feedback = a1_msgs.RobotCtrl_searchFeedback()
        
        # create result message
        self._result = a1_msgs.RobotCtrl_searchResult()
        
        # create action server
        self._act_server = actionlib.SimpleActionServer(
                                "robotCtrl_search",
                                a1_msgs.RobotCtrl_searchAction,
                                execute_cb=self.act_server_callback,
                                auto_start=False
                            )
        
        # start action server
        self._act_server.start()

    def act_server_callback(self, goal: a1_msgs.RobotCtrl_searchGoal):
        """
        Handles the goal requests to look for a target marker id

        :param: goal: goal request from the client
        :type goal: assignment.msg.RobotCtrl_searchGoal

        :return: None
        """

        # subscriber to camera topic
        self._vision_sub = rospy.Subscriber(
                                'info_vision',
                                RobotVision,
                                self.vision_callback
                            )

        # get target marker id to look for
        self._target_id = goal.id
        
        try:
            # wait for the first update from the camera to come
            self._camera_info = rospy.wait_for_message('info_vision',RobotVision, timeout=self._camera_timeout)
        
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.loginfo(e)
            return False
        
        success = self.search_markerId(self._target_id)
        self._act_server.set_succeeded(result = a1_msgs.RobotCtrl_searchResult(success))
        
        # subscription not needed anymore. Remove it
        self._vision_sub.unregister()


    def search_markerId(self, id: int):
        """
        Looks for the target marker id rotating the robot

        :param: id: id of the target marker to look for
        :type id: int

        :return: Bool: True if it finds the requested target marker id, False otherwise
        """

        r = rospy.Rate(self._rate)

        # velocity reference control
        velocity = Twist()
        velocity.angular.z = 0
        velocity.linear.x = 0

        # send velocity command
        self._ctr_cmd_vel_pub.publish(velocity)

        while(not self._camera_info.ids or id != self._camera_info.ids[0]):

            # rospy.loginfo("control - looking for marker id: %d" % id)

            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("control_act_server search - Searching goal preempted")

                # stop looking for target marker id
                velocity.angular.z = 0
                self._ctr_cmd_vel_pub.publish(velocity)

                # preempt current goal
                self._act_server.set_preempted()
                return False
            
            # send feedback on current id found by the camera
            # if self._camera_info.id != None:
            self._feedback.ids = self._camera_info.ids
            self._act_server.publish_feedback(self._feedback)

            # keep on looking
            velocity.angular.z = -0.5
            self._ctr_cmd_vel_pub.publish(velocity)

            r.sleep()
        
        # send feedback on current id found by the camera
        # if self._camera_info.id != None:
        self._feedback.ids = self._camera_info.ids
        self._act_server.publish_feedback(self._feedback)

        # stop looking for target marker id
        velocity.angular.z = 0
        self._ctr_cmd_vel_pub.publish(velocity)
        
        return True


class RobotCtrlServer_reachMarkerId(RobotCtrl_base):
    """
    Action Server controller for reaching the target marker
    within a threshold
    """
    def __init__(self):
        
        # initialize parent class
        super().__init__()

        # keep on getting close till the target marker side size
        # is lower than the threshold
        self._marker_side_th = None
        # tollerance error on marker side
        self._marker_side_tollerance = 1

        # create feedback message
        self._feedback = a1_msgs.RobotCtrl_reachFeedback()
        
        # create result message
        self._result = a1_msgs.RobotCtrl_reachResult()
        
        # create action server
        self._act_server = actionlib.SimpleActionServer(
                                "robotCtrl_reach",
                                a1_msgs.RobotCtrl_reachAction,
                                execute_cb=self.act_server_callback,
                                auto_start=False
                            )
        
        # start action server
        self._act_server.start()

    def act_server_callback(self, goal: a1_msgs.RobotCtrl_reachGoal):
        """
        Handles the goal requests to reach a target marker id

        :param: goal: goal request from the client
        :type goal: assignment.msg.RobotCtrl_reachGoal

        :return: None
        """
    
        # subscriber to camera topic
        self._vision_sub = rospy.Subscriber(
                                'info_vision',
                                RobotVision,
                                self.vision_callback
                            )

        # set target marker id used to filter out the image from the camera
        self._target_id = goal.id
        self._marker_side_th = goal.marker_side_th

        try:
            # wait for the first update from the camera to come
            self._camera_info = rospy.wait_for_message('info_vision',RobotVision, timeout=self._camera_timeout)
        
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.loginfo(e)
            return False

        success = self.reach_marker_id(self._marker_side_th)
        self._act_server.set_succeeded(result = a1_msgs.RobotCtrl_reachResult(success))
        
    
    def reach_marker_id(self, marker_side_th: float):
        """
        Gets close to the target marker up to a threshold.
        It keeps the target marker in the middle of the camera vision

        :param marker_side_th: threshold to stop the approach to the target marker
        :type marker_side_th: float

        :return: Bool: True if marker reached within the threshold, False otherwise
        """
        # check that camera info marker coordinates are not empty
        # control rate
        r = rospy.Rate(self._rate)

        marker_side = 0

        # proportional gains
        linear_gain = 0.003
        angular_gain = 0.003

        # velocity reference
        velocity = Twist()

        # check just vertical side
        x_cord = self._camera_info.marker_top_right[0] - self._camera_info.marker_bottom_right[0]
        y_cord = self._camera_info.marker_top_left[1] - self._camera_info.marker_bottom_left[1]

        marker_side = math.sqrt(math.pow(x_cord,2) + math.pow(y_cord,2))

        self._feedback.id = self._target_id
    
        while(abs(marker_side - marker_side_th) > self._marker_side_tollerance):

            # check that the goal has not been requested to be cleared
            if self._act_server.is_preempt_requested():
                rospy.loginfo("control_act_server reach - Getting close to target marker preempted")
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

            # send feedback on the current marker size
            self._feedback.marker_side = marker_side
            self._act_server.publish_feedback(self._feedback)

            # rospy.loginfo("control - marker side size error: %f" % linear_error)
            # rospy.loginfo("control - camera center error: %f" % angular_error)

            # cycle timing
            r.sleep()
        
        # target marker reached
        velocity.linear.x = 0
        velocity.angular.z = 0
        self._ctr_cmd_vel_pub.publish(velocity)

        # subscription not needed anymore. Remove it
        self._vision_sub.unregister()

        return True



def main():
    """
    Main function of the node
      - initialize the node
      - creates action server object for ``robotCtrl_search``
      - creates action server object for ``robotCtrl_reach``

    :return: None
    """
    rospy.init_node("robot_controller_action_server")
        
    ctr_act_server_searchMarkerId = RobotCtrlServer_searchMarkerId()
    ctr_act_server_reachMarkerId = RobotCtrlServer_reachMarkerId()

    rospy.loginfo("robot_controller_action_server initialized")

    rospy.spin()


if __name__ == "__main__":
    main()

