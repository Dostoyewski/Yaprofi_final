#!/usr/bin/env python
# coding: utf-8
import rospy
import threading
import time
import sys
import tf.transformations as tftr
from numpy import *

from std_msgs.msg import Empty, Float32
from geometry_msgs.msg import Pose, Point, Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from copter_control.msg import CopterTarget

lock = threading.Lock()


class DroneController:

    def __init__(self):

        "ROS stuff"
        self.marker_features_sub = rospy.Subscriber('/marker/features', CameraFeatures, self.marker_features_callback)
        self.drone_odom_sub = rospy.Subscriber('/drone/odom', Odometry, self.drone_odom_callback)
        self.drone_target_pose_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=True)
        self.drone_takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1, latch=True)
        self.drone_estop_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1, latch=True)
        self.drone_land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1, latch=True)

        "Drone state"
        self.drone_state_position = Point()
        self.drone_state_orientation = Vector3()
        self.drone_state_lin_vel = Vector3()
        self.drone_state_ang_vel = Vector3()

        "Camera features for feedback"
        self.camera_features = CameraFeatures()
        

    def drone_odom_callback(self, msg):
        """
        Odometry callback.
        Gets drone state from CoppeliaSim. Like a current position, orientation and velocities
        """
        lock.acquire()

        self.drone_state_position = msg.pose.pose.position

        q = msg.pose.pose.orientation
        roll, pitch, yaw = tftr.euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.drone_state_orientation = Vector3(roll, pitch, yaw)

        self.drone_state_lin_vel = msg.twist.twist.linear
        self.drone_state_ang_vel = msg.twist.twist.angular

        lock.release()

    def marker_features_callback(self, msg):
        """
        Features extraction callback.
        Gets features ()
        """
        lock.acquire()
        self.camera_features = msg
        lock.release()

    def stop(self):
        """ Reset the robot """
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = self.Z0
        self.drone_target_pose_pub.publish(pose)

        self.marker_features_sub.unregister()
        self.drone_odom_sub.unregister()
        self.drone_target_pose_pub.unregister()

    def update(self, dt, t):
        """ Update control signal for drone """
        ####################################################
        #### TODO WRITE YOUR SOLUTION BELOW! ###############
        #### THIS IS DRONE CONTROLLER ######################
        #### THIS CODE JUST FOR EXAMPLE ####################
        ####################################################

        self.pos_x = 0
        self.pos_y = 0

                
        pose = Pose()
        pose.position.x = self.pos_x
        pose.position.y = self.pos_y
        pose.position.z = self.Z0   # don't change this. It sets from judge node
        self.drone_target_pose_pub.publish(pose)


