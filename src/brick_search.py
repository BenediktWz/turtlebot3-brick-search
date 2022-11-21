#!/usr/bin/env python

import rospy
import roslib
import math
import cv2 as cv # OpenCV2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.srv import GetMap
import tf
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
import actionlib
import random
import copy
from threading import Lock
import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import hsv_to_rgb
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from subprocess import call



def wrap_angle(angle):
    # Function to wrap an angle between 0 and 2*Pi
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(wrap_angle(pose_2d.theta))
    pose.orientation.z = math.sin(wrap_angle(pose_2d.theta) / 2.0)

    return pose

class BrickSearch:
    def __init__(self):

        # Variables/Flags
        self.localised_ = False
        self.brick_found_ = False
        self.image_msg_count_ = 0
        self.image_pose_3D_ = 0
        self.brick_centre_ = 0
        
        
        self.cv_bridge_ = CvBridge()
        
        self.tf_listener_ = tf.TransformListener()

        # Subscribe to the camera
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.
        image_callback, queue_size=1)
        rospy.loginfo("image_callback performed")

        # Advertise "cmd_vel" publisher to control TurtleBot manually
        self.cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Action client for move_base
        self.move_base_action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action...")
        self.move_base_action_client_.wait_for_server()
        rospy.loginfo("move_base action available")


    def get_pose_2d(self):

        # Lookup the latest transform
        (trans,rot) = self.tf_listener_.lookupTransform('map', 'base_link', rospy.Time(0))

        #print('translation from TransformListener:',trans)
        #print('rotation from TransformListener',rot)

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = trans[0]
        pose.y = trans[1]

        qw = rot[3];
        qz = rot[2];

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw));

        return pose

    def amcl_pose_callback(self, pose_msg):

        # Check the covariance
        frobenius_norm = 0.0

        for e in pose_msg.pose.covariance:
            frobenius_norm += e**2

        if frobenius_norm < 0.05:
            self.localised_ = True

            # Unsubscribe from "amcl_pose" because we should only need to localise once at start up
            self.amcl_pose_sub_.unregister()


    def image_callback(self, image_msg):
        # Use this method to identify when the brick is visible

        # The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
        if self.image_msg_count_ < 3:
            self.image_msg_count_ += 1
            return
        else:
            self.image_msg_count_ = 0

        # Copy the image message to a cv_bridge image
        image = self.cv_bridge_.imgmsg_to_cv2(image_msg)
        image_RGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_hsv = cv2.cvtColor(image_RGB, cv2.COLOR_RGB2HSV)
        
        # safe the pose of the robot at the time it took the image
        image_pose_2D = self.get_pose_2d()
        image_pose_3D = pose2d_to_pose(image_pose_2D)
        
		# set the colour range in HSV colour format
        low = (111,200,20)
        high = (120, 255, 255)
        
        # create masked picture
        masked_picture = cv2.inRange(image_hsv, low, high)
        
        # count the number of pixels that are in the colour range
        mask_pixel=0
        for i in range(0,1080):
        	for j in range(0,1920):
        		if masked_picture[i,j] != 0:
        			mask_pixel += 1
        print('Number of red pixels:',mask_pixel)
        
        # this if clause is testing whether the brick was found or not
        if (mask_pixel>60000) and (self.brick_found_==False):
            
            self.brick_found_=True
			
			# stop the explorelite node
            # call(["rosnode kill /explore"], shell=True)
			
			rospy.sleep(0.1)
			
			# move robot to image_pose (pose where the brick was spotted)
			action_goal2 = MoveBaseActionGoal()
			action_goal2.goal.target_pose.header.frame_id = "map"
			print('image_pose_3D.orientation.w', image_pose_3D.orientation.w)
			print('image_pose_3D.orientation.z', image_pose_3D.orientation.z)
			
			action_goal2.goal.target_pose.pose = image_pose_3D
			self.move_base_action_client_.send_goal(action_goal2.goal)
			rospy.loginfo('Sending image goal...')
			print('Image Pose 3D:', image_pose_3D)
			
			# print out the picture that includes the brick and the correlating mask 
			plt.subplot(1, 3, 1)
			plt.imshow(image)
			plt.subplot(1, 3, 2)
			plt.imshow(masked_picture)
			
			
			# find image coordinates of the center of the brick (w,h)
			
			_,contours, _ = cv.findContours(masked_picture, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
			# Find the index of the largest contour
			areas = [cv2.contourArea(c) for c in contours]
			max_index = np.argmax(areas)
			# draw contour on image
			cv.drawContours(image, contours, max_index, (0,255,0), 10)
			
			# Find center of contour
			max_cnt=contours[max_index]
			x,y,w,h = cv2.boundingRect(max_cnt)
			center = (x+w//2, y+h//2)
			cv.drawMarker(image, center, color=(0,255,0), markerType=cv2.MARKER_CROSS, thickness=10)
			plt.subplot(1, 3, 3)
			plt.imshow(image)
			plt.show()
			rospy.loginfo('displaying_image_of_the_brick')
			print('center:', center)
			self.image_pose_3D_ = image_pose_3D
			self.brick_centre_ = center
			
		
        # You can set "brick_found_" to true to signal to "mainLoop" that you have found a brick
        # You may want to communicate more information
        # Since the "image_callback" and "main_loop" methods can run at the same time you should protect any shared variables
        # with a mutex
        # "brick_found_" doesn't need a mutex because it's an atomic
		

        rospy.loginfo('image_callback')
        rospy.loginfo('brick_found_: ' + str(self.brick_found_))

    def main_loop(self):

        # Wait for the TurtleBot to localise
        # comment out spinning part
        '''rospy.loginfo('Localising...')
        while not rospy.is_shutdown():

            # Turn slowly
            twist = Twist()
            twist.angular.z = 1.
            self.cmd_vel_pub_.publish(twist)

            if self.localised_:
                rospy.loginfo('Localised')
                break

            rospy.sleep(0.1)

        # Stop turning
        twist = Twist()
        twist.angular.z = 0.
        self.cmd_vel_pub_.publish(twist)

        # The map is stored in "map_"
        # You will probably need the data stored in "map_.info"
        # You can also access the map data as an OpenCV image with "map_image_"

        # Here's an example of getting the current pose and sending a goal to "move_base":
        pose_2d = self.get_pose_2d()
        
        # hardcoded pose interchangeable
        hardcoded_pose = Pose(Point(3, 3, 0), Quaternion(0.0, 0.0, 0.6, 0.77))

        rospy.loginfo('Current pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # Move forward 0.5 m
        pose_2d.x += 0.5 * math.cos(pose_2d.theta)
        pose_2d.y += 0.5 * math.sin(pose_2d.theta)

        rospy.loginfo('Target pose: ' + str(pose_2d.x) + ' ' + str(pose_2d.y) + ' ' + str(pose_2d.theta))

        # Send a goal to "move_base" with "self.move_base_action_client_"
        action_goal = MoveBaseActionGoal()
        action_goal.goal.target_pose.header.frame_id = "map"
        action_goal.goal.target_pose.pose = hardcoded_pose # pose2d_to_pose(pose_2d)
	
		
        rospy.loginfo('Sending image goal...')
        self.move_base_action_client_.send_goal(action_goal.goal)'''

        # This loop repeats until ROS is shutdown
        # You probably want to put all your code in here
        while not rospy.is_shutdown():

            rospy.loginfo('main_loop')

            # Get the state of the goal
            state = self.move_base_action_client_.get_state()
            # check whether goal pose where the image was taken has been reached
            if state == 3:
                reached_goal_pose_2D = self.get_pose_2d()
            	reached_goal_pose_3D = pose2d_to_pose(reached_goal_pose_2D)
            	print(reached_goal_pose_3D)
            	print('This is the pose of the robot at the time the image was taken:',self.image_pose_3D_)
            	print('Brick center has the coordinates:', self.brick_centre_)
            	rospy.sleep(2)

            	# shutdown the brick_search node
            	rospy.signal_shutdown('Brick found O_O')

            # Delay so the loop doesn't run too fast
            rospy.sleep(0.2)


if __name__ == '__main__':

    # Create the ROS node
    rospy.init_node('brick_search')

    # Create the brick search
    brick_search = BrickSearch()

    # Loop forever while processing callbacks
    brick_search.main_loop()




