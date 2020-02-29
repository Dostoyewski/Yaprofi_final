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
from bac_task.msg import CopterTarget

lock = threading.Lock()


class DroneController:

    def __init__(self):

        "ROS stuff"
        self.drone_odom_sub = rospy.Subscriber('/tello/odom', Odometry, self.drone_odom_callback)
        self.drone_target_vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1, latch=True)
        self.drone_takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1, latch=True)
        self.drone_estop_pub = rospy.Publisher('/tello/emergency', Empty, queue_size=1, latch=True)
        self.drone_land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1, latch=True)
        self.marker_features_sub = rospy.Subscriber('/start/params', CopterTarget, self.fly_params_callback)

        "Drone state"
        self.drone_state_position = Point()
        self.drone_state_orientation = Vector3()
        self.drone_state_lin_vel = Vector3()
        self.drone_state_ang_vel = Vector3()

        "Fly params"
        self.h = 0
        self.dh = 0
        self.phi = 0

        "Displacement"
        self.dx, self.dy, self.dz = 0, 0, 0

        "fly etape"
        """ 0 — взлет, 1 — набор высоты, 2 — задержка, 3 — изменение высоты,
        4 — задержка, 5 — изменение ориентации, 6 — задержка, 7 — посадка"""
        self.code = -1

        "Camera features for feedback"
        self.camera_features = CameraFeatures()

    def fly_params_callback(self, msg):
        """
        Fly params callback
        Sets h, dh, phi
        """
        lock.acquire()

        self.h = msg.h
        self.dh = msg.dh
        self.phi = msg.phi

        "Starting process"
        self.code = 0

        self.counter = 0

        print("Fly parametes:", self.h, self.dh, self.phi)

        lock.release()
        

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

    def stop(self):
        """ Reset the robot """
        e = Empty()
        self.drone_estop_pub.publish(e)

    def calc_velocity(self, x):
        '''calc velocity'''
        vel = -0.25*x**3 - 0.3*sin(x)
        if vel > 0.5:
            vel = 0.5
        if vel < -0.5:
            vel = -0.5
        return vel

    def update(self, dt, t):
        """ Update control signal for drone """

        if self.code == 0:
            "Takeoff function"
            e = Empty()
            self.drone_takeoff_pub.publish(e)
            self.code += 1
            self.dx = self.drone_state_position.x
            self.dy = self.drone_state_position.y
            self.dz = self.drone_state_position.z
            print('Takeoff OK', self.dx, self.dy, self.dz)
        
        if self.code == 1:
            vel = Twist()
            vel.linear.z = self.calc_velocity(self.drone_state_position.z - self.h)
            if abs(vel.linear.z) < 0.05:
                vel.linear.z = 0
                self.code += 1
            self.drone_target_vel_pub.publish(vel)

        if self.code == 2:
            if self.counter < 500:
                self.counter += 1
            else:
                e = Empty()
                print("Landing...")
                self.drone_land_pub.publish(e)
                self.counter += 1