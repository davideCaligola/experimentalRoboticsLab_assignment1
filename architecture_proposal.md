# Proposed architecture  
From the assignment specifications:

>The markers have the following meanings:  
>  * Marker 11 -> rotate until you find marker 12; then reach marker 12  
>  * Marker 12 -> rotate until you find marker 13; then reach marker 13  
>  * Marker 13 -> rotate until you find marker 15; then reach marker 15  
>  * Marker 15 -> done!
>
>Please notice that “reach marker xxx” means that one side of the xxx marker must be at least 200 pixels in the
camera frame!

The package will consist of three nodes:
 - [logic node](#logic-node)
 - [robot_control](#robot_control-node)
 - [camera](#camera-node)

Following a rough description of each node.


## logic node
Brain of the system. It contains a state machine which defines the behaviour of the task.  
Following a pseudo-code description of the state machine.
```
    // create list of marker to reach.
    marker_id_to_reach = [11, 12, 13, 15]
    
    switch(state)

        get_next_marker:
            // check if all markers have been found
            if marker_id_to_reach is empty
                state = finish
            else
                // extract the first element of the list  
                // of marker to look for.  
                // The element is removed from the list  
                marker_id_to_look_for = extract element from marker_id_to_reach
            
            state = looking_for_marker

        looking_for_marker:
            // ask controller to execute the action  
            // for looking for the marker with id  
            // "marker_id_to_reach"
            
            if (marker_found)
                state = getting_close_to_marker

        getting_close_to_marker:
            // ask controller to execute the action  
            // get close to the marker, maybe specifying
            // the criterion to decide when it is close
            // enough (i.e. marker size 200x200 pixels)
            
            if (robot close enough to the marker)
                state = get_next_marker
```
        finish:
            terminate the task

## robot_control node
To be decided if to be implemented as action server or service. Formally, an action server should be more correct, since it is possible to have feedback and in case stop the current executing action.  
With service it will be needed to publish always the status to make the node logic aware, but it is not possible to stop a current executing action (see [interrupting service callbacks](https://answers.ros.org/question/205631/one-callback-interrupting-another-is-that-possible/))


The control node provides the following goals to achieve:
* look_for_marker  
  it will rotate until the camera will provide the topic message marker_found = true.  
  Once the goal is reached, the result will be the marker_id  

* get_close_to_marker  
  The controller will check the topic message providing the distance between the marker center and the camera view center, for guarantee to keep the marker always in the middle of the camera view.  
  If the distance between the two centers is greater than a threshold, the robot will turn right/left to compensate it.  
  If at least one size of the marker, provided by the camera topic marker_size is 200px, the marker is considered reached. (Only one, because in the case the robot reaches the marker diagonally, the horizontal size could be less than 200px, but the vertical on could already be 200px, thus enough)


## camera node
It will publish the following topics:
* marker_found: true/false
* marker_id: integer (valid only when marker_found = true)
* marker_size: high, width
* distance between marker center and camera view center

It would be possible to consider to group together some or all topics in fewer or just one single topic containing all the informations.
