#!/usr/bin/env python

# Author: Tim (Kyoung Tae) Kim
# Date: November 9th, 2021

# Import of python modules.
import math
from enum import Enum
import numpy as np
import random

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan  # message type for scan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

import message_filters
from std_msgs.msg import Int32, Float32

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'  # name of topic for Stage simulator
# DEFAULT_SCAN_TOPIC = 'scan' # For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10  # Hz.

# Velocities that will be used
LINEAR_VELOCITY = 0.2  # m/s

# Goal distance to the wall
THRESHOLD_DISTANCE = 0.25  # m
TARGET_DISTANCE = 0.5  # m

# Field of view in radians that is checked in front of the robot
MIN_SCAN_ANGLE_RAD = - 180.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = + 180.0 / 180 * math.pi

RESOLUTION = 768


class PD:
    # controller constructor
    def __init__(self, kp, kd):
        self._p = kp      # proportional gain
        self._d = kd      # derivative gain

    # step function that calculates new velocity
    def step(self, error_list, dt):
        u = self._p * error_list[-1]
        if(len(error_list) > 1):
            u += self._d * (error_list[-1] - error_list[-2])/dt
        return u

# finite state machine


class fsm(Enum):
    INITIAL = 1
    COLLISION = 2
    TRACKING = 3
    STAND_BY = 4


class PersonTracker():

    def __init__(self, controller, linear_velocity=LINEAR_VELOCITY,
                 scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD],
                 threshold_distance=THRESHOLD_DISTANCE, target_distance=TARGET_DISTANCE):
        # Setting up publishers/subscribers.
        self._cmd_pub = rospy.Publisher(
            DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan)
        # TO DO: add subscriber to fall detection node (size of frame box)
        # x, y, z  (x, y) is center of frame box, z is the depth (distance to person )
        self._fall_sub = rospy.Subscriber("fall_detection", Point)
        # TO DO: add publisher to planner node (to track and recover target)
        ##  self._planner_pub = rospy.Publisher("planner", str, queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self._laser_sub, self._fall_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self._callback)

        # set up the controller
        self.controller = controller

        self.linear_velocity = linear_velocity    # linear velocity of robot
        self.scan_angle = scan_angle              # angle for field of view

        self.threshold_distance = threshold_distance        # minimum distance to obstacle
        self.target_distance = target_distance             # target distance to person
        self.target_direction = RESOLUTION/2                # target direction to person

        # list of errors for target distance
        self.direction_error_list = []
        self.distance_error_list = []

        # intial state of the robot
        self._fsm = fsm.INITIAL

    # move the robot
    def move(self, linear_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    # stop the robot
    def stop(self):
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _callback(self, laser_msg, fall_msg):

        ############################### Laser #####################################
        min_distance = float('-inf')

        for i in range(len(laser_msg.ranges)):
            if float('-inf') < laser_msg.ranges[i] and laser_msg.ranges[i] < float('inf'):
                min_distance = min(min_distance, laser_msg.range[i])

        # rotate if minimum distance less than goal
        if min_distance < THRESHOLD_DISTANCE:
            self._fsm = fsm.COLLISION
        else:
            self._fsm = fsm.TRACKING
        ################################ Fall #####################################

        # TO DO: calculate size of box from Point
        # x, y, z = msg.?

        # calculate the distance and append to error
        direction_error = self.target_direction - fall_msg.y
        distance_error = self.target_distance - fall_msg.z
        self.direction_error_list.append(direction_error)
        self.distance_error_list.append(distance_error)
        ###########################################################################

    def spin(self):
        rate = rospy.Rate(FREQUENCY)  # loop at 10 Hz.

        while not rospy.is_shutdown():

            # first move forward to generate some errors
            if self._fsm == fsm.INITIAL:

                start_time = rospy.get_rostime()
                duration = rospy.Duration(1)

                while not rospy.is_shutdown():

                    if rospy.get_rostime() - start_time < duration:
                        self.move(self.linear_velocity, 0)
                    else:
                        self.stop()
                        break

                    rate.sleep()

            # rotate if collision expected by the laser
            elif self._fsm == fsm.COLLISION:

                angle = random.uniform(0, math.pi)
                start_time = rospy.get_rostime()
                duration = rospy.Duration(abs(angle)/self.angular_velocity)

                while not rospy.is_shutdown():

                    if rospy.get_rostime() - start_time < duration:
                        self.move(0, np.sign(angle) * self.angular_velocity)
                    else:
                        self.stop()
                        break

                    rate.sleep()

            # otherwise rotate by angle provided by controller
            elif self._fsm == fsm.TRACKING:

                new_rotation = self.controller.step(
                    self.direction_error_list, 0.1)
                new_direction = self.controller.step(
                    self.distance_error_list, 0.1)

                self.move(new_direction, new_rotation)

            # elif self._fsm == fsm.STANDBY:
            #     # send message to planner node
            #     pass

            rate.sleep()


def main():
    """ Main Function """
    # 1st. initialization of node.
    rospy.init_node("person_tracker")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(4)

    # Initialize controller
    controller = PD(1, 10)

    # Initialization of the class for the robot following the right wall
    tracker = PersonTracker(controller)

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(tracker.stop)

    try:
        tracker.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == '__main__':
    main()

    # def _laser_callback(self, msg):

    #     # index of range for min and max field of view
    #     min_index = int((self.scan_angle[0] - msg.angle_min) / msg.angle_increment)
    #     max_index = int((self.scan_angle[1] - msg.angle_min) / msg.angle_increment)

    #     # find minimum distance in field of view
    #     min_distance = np.min(msg.ranges[min_index:max_index+1])
    #     min_distance = float('-inf')

    #     for i in range(len(msg.ranges)):
    #         if float('-inf') < msg.ranges[i] and msg.ranges[i] < float('inf'):
    #             min_distance = min(min_distance, msg.range[i])

    #     # rotate if minimum distance less than goal
    #     if min_distance < THRESHOLD_DISTANCE:
    #         self._fsm = fsm.COLLISION
    #     else:
    #         self._fsm = fsm.TRACKING

    # def _fall_callback(self, msg):

    #     # TO DO: calculate size of box from Point
    #     # x, y, z = msg.?

    #     # calculate the distance and append to error
    #     direction_error = self.target_direction - msg.y
    #     distance_error = self.target_distance - msg.z
    #     self.error_list.append(distance_error)

    #     # rotate if minimum distance less than goal
    #     if min_distance < GOAL_DISTANCE:
    #         self._fsm = fsm.ROTATE
