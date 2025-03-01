import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image  # Assuming you convert to an image-like format
import cv2
from cv_bridge import CvBridge

#Arbitrary values
ROI_TOP    = 200   
ROI_BOTTOM = 300  
ROI_LEFT   = 150   
ROI_RIGHT  = 250  

#Sample test values
SPEED = 1.0             
AVOIDANCE_TURN = 1.0    


bridge = CvBridge()
cmd_pub = None

def occupancy_grid_callback(img_msg):
 
    cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
    
    roi = cv_image[ROI_TOP:ROI_BOTTOM, ROI_LEFT:ROI_RIGHT]
    
    twist_msg = Twist()

    if np.count_nonzero(roi) > 0:
        rospy.loginfo("Obstacle detected in ROI! Executing avoidance maneuver.")
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = AVOIDANCE_TURN 
    else:
        twist_msg.linear.x = SPEED
        twist_msg.angular.z = 0.0

    cmd_pub.publish(twist_msg)

def obstacle_avoidance_node():
    global cmd_pub

    rospy.init_node('drone_obstacle_avoidance_2d', anonymous=True)

    rospy.Subscriber('/occupancy_grid_image', Image, occupancy_grid_callback)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Drone obstacle avoidance node (2D) started.")
    rospy.spin()

