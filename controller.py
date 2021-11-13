#!/usr/bin/env python3

# Author: Tim (Kyoung Tae) Kim
# Date: November 13th, 2021

# Import of python modules.
import math
from enum import Enum
import numpy as np
import random

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan  # message type for scan
import tf
import message_filters

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'  # name of topic for Stage simulator
# DEFAULT_SCAN_TOPIC = 'scan' # For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10  # Hz.

# Velocities that will be used
LINEAR_VELOCITY = 0.2  # m/s
ANGULAR_VELOCITY = math.pi/8

# Goal distance to the wall
THRESHOLD_DISTANCE = 0.05  # m
TARGET_DISTANCE = 35000  # z values
TARGET_DIRECTION = 200

# Field of view in radians that is checked in front of the robot
MIN_SCAN_ANGLE_RAD = - 180.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = + 180.0 / 180 * math.pi

RESOLUTION = 768


class PD:
    # controller constructor
    def __init__(self, kp, kd):
        self._p = kp      # proportional gain
        self._d = kd      # derivative gain

    # step function that calculates new linear/angular velocity
    def step(self, error_list, dt):
        u = self._p * error_list[-1]
        if(len(error_list) > 1):
            u += self._d * (error_list[-1] - error_list[-2])/dt
        if (len(error_list) >= 3):
            error_list.pop(0)
        return u

# finite state machine


class fsm(Enum):
    INITIAL = 1
    COLLISION = 2
    TRACKING = 3
    STAND_BY = 4


class PersonTracker():

    def __init__(self, controller, linear_velocity=LINEAR_VELOCITY,
                 angular_velocity=ANGULAR_VELOCITY, scan_angle=[
                     MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD],
                 threshold_distance=THRESHOLD_DISTANCE, target_distance=TARGET_DISTANCE, target_direction=TARGET_DIRECTION):
        # Setting up publishers/subscribers.
        self._cmd_pub = rospy.Publisher(
            DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self._laser_sub = message_filters.Subscriber(
            DEFAULT_SCAN_TOPIC, LaserScan)
        # add subscriber to fall detection node (size of frame box)
        # x, y, z  (x, y) is center of frame box, z is the depth (distance to person)
        self._fall_sub = message_filters.Subscriber("location_status", Point)
        # TO DO: add publisher to planner node (to track and recover target)
        ##  self._planner_pub = rospy.Publisher("planner", str, queue_size=1)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self._laser_sub, self._fall_sub], 1, 1, allow_headerless=True)
        self.ts.registerCallback(self._callback)

        # set up the controller
        self.controller = controller

        self.linear_velocity = linear_velocity    # linear velocity of robot
        self.angular_velocity = angular_velocity
        self.scan_angle = scan_angle              # angle for field of view

        self.threshold_distance = threshold_distance       # minimum distance to obstacle
        self.target_distance = target_distance             # target distance to person
        self.target_direction = target_direction           # target direction to person

        # list of errors for target distance
        self.direction_error_list = []
        self.distance_error_list = []
        self.x_values = []
        self.z_values = []

        # intial state of the robot
        self._fsm = fsm.INITIAL
        self.fall_msg = None
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
                min_distance = min(min_distance, laser_msg.ranges[i])

        # print("min distance", min_distance)
        # rotate if minimum distance less than goal
        # if min_distance < THRESHOLD_DISTANCE:
        #     self._fsm = fsm.COLLISION
        # else:
        #     self._fsm = fsm.TRACKING

        ################################ Fall #####################################

        # store fall msg and x, z values
        self.fall_msg = fall_msg
        self.z_values.append(fall_msg.z)
        self.x_values.append(fall_msg.x)
        if len(self.z_values) > 30:
            self.z_values.pop(0)

        if len(self.x_values) > 30:
            self.x_values.pop(0)

        print("x, y value", fall_msg.x, fall_msg.y)

        # calculate the distance and append to error
        direction_error = self.target_direction - \
            sum(self.x_values)/len(self.x_values)
        distance_error = self.target_distance - \
            sum(self.z_values)/len(self.z_values)
        self.direction_error_list.append(direction_error)
        self.distance_error_list.append(distance_error)

        ###########################################################################

    def spin(self):
        rate = rospy.Rate(FREQUENCY)  # loop at 10 Hz.

        while not rospy.is_shutdown():
            print("FSM Mode: ", self._fsm)

            # wait until errors have been generated
            if self._fsm == fsm.INITIAL:
                if len(self.x_values) > 0:
                    self._fsm = fsm.TRACKING

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
                self._fsm = fsm.TRACKING

            # otherwise rotate by angle provided by controller
            elif self._fsm == fsm.TRACKING:

                # calculation by PD controller
                new_rotation = self.controller.step(
                    self.direction_error_list, 0.1)
                new_velocity = self.controller.step(
                    self.distance_error_list, 0.1)
                status = self.fall_msg.action_status

                if ((not self.fall_msg) or self.fall_msg.x == 0 or
                        status == "Fall Down" or status == "Lying Down"):
                    self.stop()
                else:
                    print("Linear Velocity: ", max(0, new_velocity/80000))
                    print("Angular Velocity: ", new_rotation/200)
                    self.move(max(0, new_velocity/80000), new_rotation/200)
                    self.fall_msg = None
            rate.sleep()


def main():
    """ Main Function """
    # 1st. initialization of node.
    rospy.init_node("person_tracker")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialize controller
    controller = PD(1, 2)

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
