#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo

center_cam_x = 0
center_cam_y = 0

def camera_cb(camera_msg):

    global center_cam_x, center_cam_y

    center_cam_x = camera_msg.width / 2
    center_cam_y = camera_msg.height / 2

def img_cb(img_msg):

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    
    corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    

    if ids is not None:
    
        center_marker_x = (corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]) / 4
        center_marker_y = (corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]) / 4
    	
        print(ids)
        print(center_cam_x)
        print(center_cam_y)
        print(center_marker_x)  # calcolo centro marker da verificare
        print(center_marker_y)
        
    else:
    	print("None")
        

def main():

    
    rospy.Subscriber('/camera/color/image_raw', Image, img_cb)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_cb)
    
    #da inserire un publisher al nodo control che invia l'id, il centro del marker e il centro della camera (topic: vision_info)
    
    rospy.spin()

if __name__=='__main__':

	try:
		rospy.init_node('robot_vision') 
		
		main()
		
		
	except rospy.ROSInterruptException:
		
		print("Error client")
		exit()
