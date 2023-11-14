#! /usr/bin/env python

from matplotlib.scale import LogisticTransform
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import assignment_1.msg as a1_msgs
from assignment_1.msg import LogicState, ControlAction

ctr_goal_status_sub = None  # subscriber to get controller goal status
ctr_client = None           # client of control action server
state = None                # state machine status
marker_side_th = 175        # threshold for limit on getting close to the marker
current_marker_id = None    # marker id currently looked for
cycle_rate = None

def search_marker_id_active_cb():
    """
    Updates the logic state machine status to wait to find the target marker
    when the search goal becomes active    

    :return: None
    """
    global state

    state = LogicState.LOOKING_FOR_ID

def search_marker_id_done_cb(goalStatus: GoalStatus, result: a1_msgs.RobotControllerResult):
    """
    Updates the logic state machine status to send the get close to the
    target market goal when the target marker id is found

    :param goalStatus: goal status of the current goal
    :type goalStatus: GoalStatus
    
    :return: None
    """
    
    global state

    if (result.success):
        state = LogicState.SEND_GET_CLOSE
    else:
        rospy.loginfo("************* ERROR ****************")
        rospy.loginfo("Something went wrong while looking for the target marker id")
        state = LogicState.FINISH

def get_close_to_marker_active_cb():
    """
    Updates the logic state machine status to wait to get close to the target marker
    when the search goal becomes active    

    :return: None
    """
    global state

    state = LogicState.GETTING_CLOSE

def get_close_to_marker_done_cb(goalStatus: GoalStatus, result: a1_msgs.RobotControllerResult):
    """
    Updates the logic state machine status to send the id of the new target marker
    when the robot is close enough to the current target marker

    :param goalStatus: goal status of the current goal
    :type goalStatus: GoalStatus

    :return: None
    """
    
    global state
    
    if (result.success):
        state = LogicState.SEND_ID
    else:
        rospy.loginfo("************* ERROR ****************")
        rospy.loginfo("Something went wrong while getting close to the target marker")
        state = LogicState.FINISH

def terminate_node():
    """
    Sends the shutdown signal to the node

    :return: None
    """
    rospy.signal_shutdown("\nShutting down node on user request...")


def start(ids: list):
    """
    Defines the state machine for handling the different phases of the process:

      * 0: select and send the target marker id to look for
      * 1: waiting for finding target marker id
      * 2: send command to get closer to target marker id
      * 3: waiting for getting close to marker id
      * 4: terminate the node

    :param ids: list of marker id to look for
    :type ids: list

    :return: None
    """

    global ctr_client, state, current_marker_id, cycle_rate

    # passing an empty list finishes the process
    if ids:
        state = LogicState.SEND_ID
    else:
        state = LogicState.FINISH

    while(not rospy.is_shutdown()):

        # select target marker id and send request
        # to search for such a marker id
        if state == LogicState.SEND_ID:
            if ids:
                current_marker_id = ids.pop(0)
                # create goal for looking for target marker id
                ctr_goal = a1_msgs.RobotControllerGoal(
                                action = ControlAction.SEARCH_ID,
                                id = current_marker_id
                            )

                # send the goal to action server
                ctr_client.send_goal(ctr_goal,
                                     active_cb = search_marker_id_active_cb,
                                     done_cb = search_marker_id_done_cb
                                    )

                state = LogicState.WAIT_FOR_GOAL_SEARCH_ACTIVE
            
            # all target markers have been reached
            else:
                state = LogicState.FINISH

        # wait for target marker id to be found
        elif state == LogicState.LOOKING_FOR_ID:
            # waiting for finding target marker id
            pass

        # send request to get close to target marker
        elif state == LogicState.SEND_GET_CLOSE:
            # create goal for getting close to target marker
            ctr_goal = a1_msgs.RobotControllerGoal(
                            action=ControlAction.GET_CLOSE,
                            marker_side_th = marker_side_th,
                            id = current_marker_id
                        )

            # send goal to action server
            ctr_client.send_goal(ctr_goal,
                                 active_cb = get_close_to_marker_active_cb,
                                 done_cb = get_close_to_marker_done_cb
                                )

            state = LogicState.WAIT_FOR_GOAL_GET_CLOSE_ACTIVE

        elif state == LogicState.GETTING_CLOSE:
            # waiting for getting close enough to the marker
            pass

        elif state == LogicState.FINISH:
            terminate_node()

        elif state == LogicState.WAIT_FOR_GOAL_SEARCH_ACTIVE:
            # waiting for activating goal for searching target marker id
            pass

        elif state == LogicState.WAIT_FOR_GOAL_GET_CLOSE_ACTIVE:
            # wiating for activating goal for getting close to target marker
            pass
        
        # exception handling for rate whne the process is terminated
        try:
            # keep cycle time
            cycle_rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            print(e)


def main():
    """
    Main function of the node
      - initialize the node
      - creates client for action server ``robotController``
      - creates subscriber for topic ``/robotController/status`` of robot controller
      - launches function `start` with the list of marker ids to look for. This function
        handles the process phases.

    :return: None
    """

    global ctr_goal_status_sub, ctr_client, cycle_rate

    # initialize node
    rospy.init_node("robot_logic")
    
    # client for controller
    ctr_client = actionlib.SimpleActionClient(
                    "robotController",
                    a1_msgs.RobotControllerAction
                )

    rospy.loginfo("Waiting for action server 'robotController'...")
    
    # wait for the server controller to be started
    ctr_client.wait_for_server()

    rospy.loginfo("action server 'robotController' found")

    # ordered list of marker ids to look for
    # ids = [11, 12, 13, 15]
    ids = [11, 12]

    # get process rate
    freq_rate = rospy.get_param("process_rate")
    # initialize logic rate
    cycle_rate = rospy.Rate(freq_rate)

    rospy.loginfo("robot_logic node initialized")

    # start looking for markers
    start(ids)

    rospy.spin()


if __name__ == "__main__":
    main()
        
