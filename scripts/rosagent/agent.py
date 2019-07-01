"""
This module contains the base classes for an individual agent in a multi
agent setup
"""

import json
from math import acos, asin, atan
from pprint import pprint
import sys
import numpy as np
import random
import os
import inspect
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Agent(object):
    """
    This is the base class for all agents and related operations
    """

    def __init__(self, file_name, data_folder):
        # initialize the various known lists
        self.agent_names = ['red', 'blue', 'green', 'violet']
        self.agent_types = ['Turtlebot']
        self.pub_classes = ['Twist', 'Pose', 'String']
        self.sub_classes = ['Odometry', 'String', 'LaserScan']
        self.simple_inits = ['_simple_init']
        self.complex_inits = ['_complex_init']
        self.multi_inits = ['_multi_init']

        # store the file name and extract its data
        self.agent_file = file_name
        with open(file_name) as json_file:
            data = json.load(json_file)
        self.agent_data = data
        self.init_type = self.agent_data['init_type']

        # initialize some common attributes to None for readability
        self.data_folder = data_folder

        # number of different types agents
        self.num_agents = {}

        # number of time steps required for completion
        self.time_steps = self.agent_data['time_steps']

        # information to initialize any node
        self.node_name = None
        self.node_anon = None
        self.node_space = None
        self.current_agent = None
        self.save_folder = ""
        self.save_objects = None

        # publishing related data
        self.pub_key = None
        self.pub_data = None
        self.pub_topic_name = None
        self.pub_class_call = None
        self.pub_class_obj = None
        self.queue_size = None
        self.pub = None
        self.pub_command = None
        self.current_publisher = None

        # subscription related data
        self.sub_key = None
        self.sub_data = None
        self.sub_topic_name = None
        self.sub_class_obj = None
        self.callback = None
        self.sub = None
        self.current_subscriber = None

        # odometry data
        self.odom_dcm = None
        self.odom_cov_positions = None
        self.odom_cov_velocities = None
        self.odom_positions = None
        self.odom_quaternions = None
        self.odom_velocities = None
        self.odom_timestamps = None

        # laser scan data
        self.scan_values = None
        self.scan_data = None
        self.scan_timestamps = None

        # tolerance for any floating point operations
        self.tolerance = 1e-3


class Turtlebot(Agent):
    """
    This class contains methods and variables to control a Turtlebot
    """

    def __init__(self, file_name, data_folder):
        # call the base class to perform sanity checks
        Agent.__init__(self, file_name, data_folder)

        # Initialize the relevant values
        self.num_agents[self.agent_data['agent_type']] = 1
        self.node_anon = self.agent_data['anonymous_node']
        self.node_space = self.agent_data['node_space']
        self.node_name = self.node_space + self.agent_data['node_name']

        # callback support parameters
        self.odom_callback_count = 0
        self.laser_callback_count = 0

        # Initialize the node
        # Note: Only one node can be initialized per file
        # Do NOT shutdown this node unless absolutely necessary
        rospy.init_node(self.node_name, anonymous=self.node_anon)

        # tell user how to stop TurtleBot
        # also note function to call when you ctrl + c
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)

        # call the complex or simple initializiation based on type
        # getattr is a function that converts a string to a Python object
        # This is useful when utilizing data from a json file
        getattr(self, self.agent_data['init_type'])()

    def _complex_init(self):
        """Internal function to initialize turtlebot via a complex json"""
        self.pub_key = self.agent_data['pub_type']
        self.sub_key = self.agent_data['sub_type']
        self.current_agent = self.agent_data['agent_name']

        if self.pub_key in self.agent_data:
            getattr(self, '_' + self.pub_key +
                    self.agent_data['init_type'])(True)

        if self.sub_key in self.agent_data:
            getattr(self, '_' + self.sub_key +
                    self.agent_data['init_type'])(True)

    def _simple_init(self):
        """Internal function to initialize turtlebot via a simple json"""
        self.current_agent = self.agent_data['agent_name']

        getattr(self, '_' + self.agent_data['comm_type'] +
                self.agent_data['init_type'])(True)

    def _publish_complex_init(self, initialize):
        """Internal function to publish complex data"""
        publishing_data = self.agent_data[self.pub_key]

        if initialize:
            # dictionary to contain contents of each publisher
            self.pub_data = {}
            self.pub_topic_name = {}
            self.pub_class_call = {}
            self.pub_class_obj = {}
            self.queue_size = {}
            self.pub = {}
            self.pub_command = {}

            for publisher in publishing_data:
                self.current_publisher = publisher
                publisher_data = publishing_data[publisher]

                self.pub_data[publisher] = publisher_data['comm_data']

                self.pub_topic_name[publisher] = (
                    publisher_data['namespace'] + '/' +
                    publisher_data['data_topic_name'])

                self.pub_class_obj[publisher] = getattr(
                    sys.modules[__name__], publisher_data['data_class_name'])

                self.pub_class_call[publisher] = getattr(
                    sys.modules[__name__], publisher_data['data_class_name'])()

                self.queue_size[publisher] = publisher_data['queue_size']

                self.pub[publisher] = rospy.Publisher(
                    self.pub_topic_name[publisher],
                    self.pub_class_obj[publisher],
                    queue_size=self.queue_size[publisher])

                self.pub_command[publisher] = getattr(
                    self, self.pub_data[publisher])(
                        self.pub_class_call[publisher],
                        publisher_data[self.pub_data[publisher]])

        else:
            for publisher in publishing_data:
                self.current_publisher = publisher
                publisher_data = publishing_data[publisher]
                self.pub[publisher].publish(self.pub_command[publisher])

    def _publish_simple_init(self, initialize):
        """Internal function to publish simple data"""
        publishing_data = self.agent_data['publish']
        self.current_publisher = self.agent_data['comm_type']

        if initialize:
            self.pub_data = publishing_data['comm_data']
            self.pub_topic_name = (publishing_data['namespace'] + '/' +
                                   publishing_data['data_topic_name'])
            self.pub_class_obj = getattr(sys.modules[__name__],
                                         publishing_data['data_class_name'])
            self.pub_class_call = getattr(sys.modules[__name__],
                                          publishing_data['data_class_name'])()
            self.queue_size = publishing_data['queue_size']
            self.pub = rospy.Publisher(self.pub_topic_name,
                                       self.pub_class_obj,
                                       queue_size=self.queue_size)
            self.pub_command = getattr(self, self.pub_data)(
                self.pub_class_call, publishing_data[self.pub_data])
        else:
            self.pub.publish(self.pub_command)

    def _subscribe_complex_init(self, initialize):
        """Internal function to subscribe to complex data"""
        subscribing_data = self.agent_data[self.sub_key]

        if initialize:
            # dictionary to contain contents of each subscriber
            self.sub_data = {}
            self.sub_topic_name = {}
            self.sub_class_obj = {}
            self.sub = {}
            self.callback = {}

            self.odom_dcm = {}
            self.odom_cov_positions = {}
            self.odom_cov_velocities = {}
            self.odom_positions = {}
            self.odom_quaternions = {}
            self.odom_velocities = {}
            self.odom_timestamps = {}

            self.scan_values = {}
            self.scan_data = {}
            self.scan_timestamps = {}

            for subscriber in subscribing_data:
                self.current_subscriber = subscriber
                subscriber_data = subscribing_data[subscriber]

                self.sub_data[subscriber] = subscriber_data['comm_data']

                self.sub_topic_name[subscriber] = (
                    subscriber_data['namespace'] + '/' +
                    subscriber_data['data_topic_name'])

                self.sub_class_obj[subscriber] = getattr(
                    sys.modules[__name__], subscriber_data['data_class_name'])

                self.odom_dcm[subscriber] = []
                self.odom_cov_positions[subscriber] = []
                self.odom_cov_velocities[subscriber] = []
                self.odom_positions[subscriber] = []
                self.odom_quaternions[subscriber] = []
                self.odom_velocities[subscriber] = []
                self.odom_timestamps[subscriber] = []
                self.scan_values[subscriber] = []
                self.scan_timestamps[subscriber] = []
                self.callback[subscriber] = getattr(
                    self, subscriber_data['callback'])
                self.sub[subscriber] = rospy.Subscriber(
                    self.sub_topic_name[subscriber],
                    self.sub_class_obj[subscriber],
                    self.callback[subscriber])
        else:
            for subscriber in subscribing_data:
                self.current_subscriber = subscriber
                subscriber_data = subscribing_data[subscriber]
                # This spin is weird
                # There normally needs to be a spin function that lets all the
                # callbacks called for the subscribers. However, in case of a
                # multi subscriber case like this one, it doesn't seem to work
                rospy.spin()

    def _subscribe_simple_init(self, initialize):
        """Internal function to subscribe to simple data"""
        subscribing_data = self.agent_data['subscribe']
        self.current_subscriber = self.agent_data['comm_type']

        if initialize:
            self.odom_dcm = []
            self.odom_cov_positions = []
            self.odom_cov_velocities = []
            self.odom_positions = []
            self.odom_quaternions = []
            self.odom_velocities = []
            self.odom_timestamps = []

            self.scan_values = []
            self.scan_timestamps = []

            self.sub_data = subscribing_data['comm_data']
            self.sub_topic_name = (subscribing_data['namespace'] + '/' +
                                   subscribing_data['data_topic_name'])
            self.sub_class_obj = getattr(sys.modules[__name__],
                                         subscribing_data['data_class_name'])
            self.callback = getattr(self, subscribing_data['callback'])
            self.sub = rospy.Subscriber(self.sub_topic_name,
                                        self.sub_class_obj,
                                        self.callback)
        else:
            rospy.spin()

    def _constant_velocities(self, data_class, input_velocities):
        """Internal function to move the Turtlebot with a constant velocity"""
        move_command = data_class

        # set the linear velocities
        move_command.linear.x = input_velocities['linear'][0]
        move_command.linear.y = input_velocities['linear'][1]
        move_command.linear.z = input_velocities['linear'][2]

        # set the angular velocities
        move_command.angular.x = input_velocities['angular'][0]
        move_command.angular.y = input_velocities['angular'][1]
        move_command.angular.z = input_velocities['angular'][2]

        return move_command

    def _talk(self, data_class_name, input_string):
        """Internal function to move the Turtlebot with a constant velocity"""
        if 'phrase' in input_string:
            talk_command = input_string['phrase']
        elif 'variable' in input_string:
            talk_command = str(getattr(self, input_string['variable']))

        return talk_command

    def _brake(self):
        """Internal function to brake the Turtlebot"""
        pass

    def _accelerate(self):
        """Internal function to accelerate the Turtlebot"""
        pass

    def _listener_callback(self, talker_phrase):
        """Listen to a phrase said by talker"""
        # subscribe to the right publisher
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', talker_phrase.data)

    def _quat_to_dcm(self, quaternion):

        """return the directional cosine matrix from quaternions
        url: https://www.mathworks.com/help/aeroblks/quaternionstodirectioncosinematrix.html
        """
        dcm11 = (quaternion[0]**2 + quaternion[1]**2 -
                 quaternion[2]**2 - quaternion[3]**2)
        dcm12 = 2*(quaternion[1]*quaternion[2] + quaternion[0]*quaternion[3])
        dcm13 = 2*(quaternion[1]*quaternion[3] - quaternion[0]*quaternion[2])
        dcm21 = 2*(quaternion[1]*quaternion[2] - quaternion[0]*quaternion[3])
        dcm22 = (quaternion[0]**2 - quaternion[1]**2 +
                 quaternion[2]**2 - quaternion[3]**2)
        dcm23 = 2*(quaternion[2]*quaternion[3] + quaternion[0]*quaternion[1])
        dcm31 = 2*(quaternion[1]*quaternion[3] + quaternion[0]*quaternion[2])
        dcm32 = 2*(quaternion[2]*quaternion[3] - quaternion[0]*quaternion[1])
        dcm33 = (quaternion[0]**2 - quaternion[1]**2 -
                 quaternion[2]**2 + quaternion[3]**2)
        directional_cosine_matrix = np.matrix([[dcm11, dcm12, dcm13],
                                               [dcm21, dcm22, dcm23],
                                               [dcm31, dcm32, dcm33]])

        return directional_cosine_matrix

    def _angles_from_quat(self, quaternion):
        """gets the euler angles from the quaternions"""
        q0 = quaternion[0]
        q1 = quaternion[1]
        q2 = quaternion[2]
        q3 = quaternion[3]
        phi = np.arctan2(
            (2*((q0*q1) + (q2*q3))),
            (1 - 2*(q1**2 + q2**2))
        )
        theta = asin(
            2*((q0*q2) - (q3*q1))
        )
        psi = np.arctan2(
            (2*((q0*q3) + (q1*q2))),
            (1 - 2*(q2**2 + q3**2))
        )
        return phi, theta, psi

    def _angle_from_dcm(self, given_mat):
        """Get only one angle from the DCM"""
        if (given_mat[0][0, 0] > 0) and (given_mat[1][0, 0] > 0):
            # first quadrant
            angle = acos(given_mat[0][0, 0])
        elif (given_mat[0][0, 0] < 0) and (given_mat[1][0, 0] > 0):
            # second quadrant
            angle = acos(given_mat[0][0, 0])
        elif (given_mat[0][0, 0] > 0) and (given_mat[1][0, 0] < 0):
            # fourth quadrant
            angle = asin(given_mat[0][0, 0])
        else:
            # third quadrant
            angle = asin(given_mat[0][0, 0])
        return angle

    def _log_odom_data(self, measures, quaternions, rot_mat):
        """Logs all the relevant odometery data"""

        rospy.loginfo(
            'x position: ' + str(measures.pose.pose.position.x) +
            ', y position: ' + str(measures.pose.pose.position.y) +
            ', z position: ' + str(measures.pose.pose.position.z))

        rospy.loginfo(
            'x linear: ' + str(measures.twist.twist.linear.x) +
            ', y linear: ' + str(measures.twist.twist.linear.y) +
            ', z linear: ' + str(measures.twist.twist.linear.z))

        rospy.loginfo(
            'x angular: ' + str(measures.twist.twist.angular.x) +
            ', y angular: ' + str(measures.twist.twist.angular.y) +
            ', z angular: ' + str(measures.twist.twist.angular.z))

        rospy.loginfo(
            'phi roll: ' + str(quaternions[0]) +
            ', theta pitch: ' + str(quaternions[1]) +
            ', psi yaw: ' + str(quaternions[2]))

    def _odometry_callback(self, odom_measure):
        """Append odometry data from the appropriate topic"""

        quat = [odom_measure.pose.pose.orientation.w,
                odom_measure.pose.pose.orientation.x,
                odom_measure.pose.pose.orientation.y,
                odom_measure.pose.pose.orientation.z]
        phi, theta, psi = self._angles_from_quat(quat)
        dcm = self._quat_to_dcm(quat)

        cov_positions = np.matrix(odom_measure.pose.covariance)
        cov_positions = cov_positions.reshape((6, 6))

        cov_velocities = np.matrix(odom_measure.twist.covariance)
        cov_velocities = cov_velocities.reshape((6, 6))

        self._log_odom_data(odom_measure, quat, dcm)

        if self.init_type in self.simple_inits:

            self.odom_positions.append([
                odom_measure.pose.pose.position.x,
                odom_measure.pose.pose.position.y,
                odom_measure.pose.pose.position.z,
                phi,
                theta,
                psi
            ])

            self.odom_quaternions.append([
                odom_measure.pose.pose.orientation.w,
                odom_measure.pose.pose.orientation.x,
                odom_measure.pose.pose.orientation.y,
                odom_measure.pose.pose.orientation.z
            ])

            self.odom_velocities.append([
                odom_measure.twist.twist.linear.x,
                odom_measure.twist.twist.linear.y,
                odom_measure.twist.twist.linear.z,
                odom_measure.twist.twist.angular.x,
                odom_measure.twist.twist.angular.y,
                odom_measure.twist.twist.angular.z
            ])

            self.odom_dcm.append(dcm)

            self.odom_cov_positions.append(cov_positions)

            self.odom_cov_velocities.append(cov_velocities)

            self.odom_timestamps.append([odom_measure.header.stamp.secs,
                                         odom_measure.header.stamp.nsecs])

        elif self.init_type in self.complex_inits:

            self.odom_positions[self.current_subscriber].append([
                odom_measure.pose.pose.position.x,
                odom_measure.pose.pose.position.y,
                odom_measure.pose.pose.position.z,
                phi,
                theta,
                psi
            ])

            self.odom_quaternions[self.current_subscriber].append([
                odom_measure.pose.pose.orientation.w,
                odom_measure.pose.pose.orientation.x,
                odom_measure.pose.pose.orientation.y,
                odom_measure.pose.pose.orientation.z
            ])

            self.odom_velocities[self.current_subscriber].append([
                odom_measure.twist.twist.linear.x,
                odom_measure.twist.twist.linear.y,
                odom_measure.twist.twist.linear.z,
                odom_measure.twist.twist.angular.x,
                odom_measure.twist.twist.angular.y,
                odom_measure.twist.twist.angular.z
            ])

            self.odom_dcm[self.current_subscriber].append(dcm)

            self.odom_cov_positions[self.current_subscriber].append(cov_positions)

            self.odom_cov_velocities[self.current_subscriber].append(cov_velocities)

    def _closest_position(self, scan_measure):
        # laser scan data for each time
        scan_data = {}
        scan_data['depths'] = []
        scan_data['angles'] = []
        scan_data['intents'] = []
        scan_data['closest'] = []
        scan_data['position'] = []
        scan_data['theta'] = []
        scan_data['data_matrix'] = []

        # Build a depths array to rid ourselves of any nan data inherent in
        # scan.ranges.
        for value in scan_measure.intensities:
            scan_data['intents'].append(value)

        # scan_measure.ranges is a tuple, and we want an array.
        full_depth_array = scan_measure.ranges[:]

        for depth in full_depth_array:
            theta = ((scan_measure.angle_increment *
                      full_depth_array.index(depth)) +
                     scan_measure.angle_min)
            if not np.isnan(depth):
                scan_data['depths'].append(depth)
                scan_data['angles'].append(theta)

        # If 'depths' is empty, object is too close for a reading
        # Thus establish our distance/position to nearest object as "0".
        if len(scan_data['depths']) == 0:
            scan_data['closest'] = 0
            scan_data['position'] = 0
            scan_data['theta'] = 0
        else:
            scan_data['closest'] = min(scan_data['depths'])
            scan_data['position'] = full_depth_array.index(
                scan_data['closest'])
            scan_data['theta'] = ((scan_measure.angle_increment *
                                   scan_data['position']) +
                                  scan_measure.angle_min)
            scan_data['data_matrix'].append(
                [scan_data['closest'], scan_data['theta']])

        # Add a log message, so that we know what's going on
        rospy.loginfo('position (real): {0}'.format(
            scan_data['position']))
        rospy.loginfo('Distance (real): {0}, theta: {1}'.format(
            scan_data['closest'], scan_data['theta']))

        return scan_data

    def _complete_position(self, scan_measure):
        # laser scan data for each time
        scan_data = {}
        scan_data['depths'] = []
        scan_data['angles'] = []
        scan_data['intents'] = []
        scan_data['data_matrix'] = []

        for value in scan_measure.intensities:
            scan_data['intents'].append(value)

        # scan_measure.ranges is a tuple, and we want an array.
        full_depth_array = scan_measure.ranges[:]

        for depth in full_depth_array:
            theta = ((scan_measure.angle_increment *
                      full_depth_array.index(depth)) +
                     scan_measure.angle_min)
            if not np.isnan(depth):
                scan_data['depths'].append(depth)
                scan_data['angles'].append(theta)

        scan_data['data_matrix'].append(
            [scan_data['depths'], scan_data['angles']])

        return scan_data

    def _laser_callback(self, scan_measure):
        # determines the closest thing to the Robot.

        # get the current position and store it
        position_type = (
            '_' +
            self.agent_data[self.current_subscriber]['position_type'] +
            '_position')
        current_position = getattr(self, position_type)(scan_measure)

        if self.init_type in self.simple_inits:
            self.scan_values.append(current_position['data_matrix'])
            self.scan_timestamps.append(scan_measure.header.stamp)
        elif self.init_type in self.complex_inits:
            self.scan_values[self.current_subscriber].append(
                current_position['data_matrix'])
            self.scan_timestamps[self.current_subscriber].append(scan_measure.header.stamp)

    def _save_odom_cov_positions(self, var, var_obj):
        """Saves the values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for mat in var_obj:
                    for row in mat:
                        out_string = (str(row[0, 0]) + ',' +
                                      str(row[0, 1]) + ',' +
                                      str(row[0, 2]) + ',' +
                                      str(row[0, 3]) + ',' +
                                      str(row[0, 4]) + ',' +
                                      str(row[0, 5]) + '\n')
                        outfile.write(out_string)

                    blank_string = '\n'
                    outfile.write(blank_string)

    def _save_odom_cov_velocities(self, var, var_obj):
        """Saves the values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for mat in var_obj:
                    for row in mat:
                        out_string = (str(row[0, 0]) + ',' +
                                      str(row[0, 1]) + ',' +
                                      str(row[0, 2]) + ',' +
                                      str(row[0, 3]) + ',' +
                                      str(row[0, 4]) + ',' +
                                      str(row[0, 5]) + '\n')
                        outfile.write(out_string)

                    blank_string = '\n'
                    outfile.write(blank_string)

    def _save_odom_dcm(self, var, var_obj):
        """Saves the values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for mat in var_obj:
                    for row in mat:
                        out_string = (str(row[0, 0]) + ',' +
                                      str(row[0, 1]) + ',' +
                                      str(row[0, 2]) + '\n')
                        outfile.write(out_string)

                    blank_string = '\n'
                    outfile.write(blank_string)

    def _save_odom_positions(self, var, var_obj):
        """Saves the values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for item in var_obj:
                    x_position = item[0]
                    y_position = item[1]
                    z_position = item[2]
                    phi = item[3]
                    theta = item[4]
                    psi = item[5]
                    out_string = (str(x_position) + ',' +
                                  str(y_position) + ',' +
                                  str(z_position) + ',' +
                                  str(phi) + ',' +
                                  str(theta) + ',' +
                                  str(psi) + '\n')
                    outfile.write(out_string)

    def _save_odom_quaternions(self, var, var_obj):
        """Saves the values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for item in var_obj:
                    w_quaternion = item[0]
                    x_quaternion = item[1]
                    y_quaternion = item[2]
                    z_quaternion = item[3]
                    out_string = (str(w_quaternion) + ',' +
                                  str(x_quaternion) + ',' +
                                  str(y_quaternion) + ',' +
                                  str(z_quaternion) + '\n')
                    outfile.write(out_string)

    def _save_odom_velocities(self, var, var_obj):
        """Saves the velocity values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for item in var_obj:
                    x_linear = item[0]
                    y_linear = item[1]
                    z_linear = item[2]
                    x_angular = item[3]
                    y_angular = item[4]
                    z_angular = item[5]
                    out_string = (str(x_linear) + ',' +
                                  str(y_linear) + ',' +
                                  str(z_linear) + ',' +
                                  str(x_angular) + ',' +
                                  str(y_angular) + ',' +
                                  str(z_angular) + '\n')
                    outfile.write(out_string)

    def _save_odom_timestamps(self, var, var_obj):
        """Saves the values generated by odometer"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for item in var_obj:
                    out_string = (str(item.secs) + ',' +
                                  str(item.nsecs) + '\n')
                    outfile.write(out_string)

    def _save_scan_timestamps(self, var, var_obj):
        """Saves the values generated by laser scanner"""
        self.odom_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

        if self.odom_callback_count > 0:
            with open(data_file, 'w') as outfile:
                for item in var_obj:
                    out_string = (str(item.secs) + ',' +
                                  str(item.nsecs) + '\n')
                    outfile.write(out_string)

    def _save_scan_values(self, var, var_obj):
        """Saves the values generated by laser scan"""
        self.laser_callback_count = np.shape(var_obj)[0]
        folder = self.save_folder
        time_step = 0

        # reorganize data
        range_cloud = var_obj[time_step][0][0]
        angle_cloud = var_obj[time_step][0][1]

        if type(range_cloud) is list:
            while time_step < self.laser_callback_count:
                data_folder = os.path.join(folder, self.current_agent + '_' + var)
                try:
                    os.mkdir(data_folder)
                except OSError:
                    print 'Either folder exists or it cannot be created'

                index = time_step + 1
                format_string = '%04d' % index
                data_file = os.path.join(data_folder, format_string + '.csv')

                with open(data_file, 'w') as outfile:
                    count = 0
                    while count < len(range_cloud):
                        out_string = (str(range_cloud[count]) +',' +
                                      str(angle_cloud[count]) + '\n')
                        outfile.write(out_string)
                        count += 1

                time_step += 1

                range_cloud = var_obj[time_step][0][0]
                angle_cloud = var_obj[time_step][0][1]

        elif type(range_cloud) is float:
            data_file = os.path.join(folder, self.current_agent + '_' + var + '.csv')

            with open(data_file, 'w') as outfile:
                while time_step < self.laser_callback_count:
                    # get the correct data
                    range_closest = var_obj[time_step][0][0]
                    angle_closest = var_obj[time_step][0][1]

                    out_string = (str(range_closest) +',' +
                                  str(angle_closest) + '\n')
                    outfile.write(out_string)
                    time_step += 1

    def save_data(self, folder=os.path.join('data', '')):
        """Function to save data from a subscriber"""
        self.save_folder = folder
        self.save_objects = ['odom_dcm', 'odom_positions', 'odom_quaternions',
                             'odom_cov_positions', 'odom_cov_velocities',
                             'odom_velocities', 'odom_timestamps',
                             'scan_timestamps', 'scan_values']
        for var in self.save_objects:
            var_obj = getattr(self, var)
            var_func = '_save_' + var
            if var_obj:
                getattr(self, var_func)(var, var_obj)

    def publish(self):
        """Method to publish and/or subscribe to ROS topics"""
        # publish or subscribe
        getattr(self, '_publish' + self.agent_data['init_type'])(False)
        pub_rate = rospy.Rate(10)
        pub_rate.sleep()

    def subscribe(self):
        """Method to publish and/or subscribe to ROS topics"""
        # publish or subscribe
        getattr(self, '_subscribe' + self.agent_data['init_type'])(False)

    def shutdown(self):
        """Function called by ROS when initiating shutdown of a node"""

        # save the relevant data
        self.save_data(self.data_folder)

        # make shutting things down fun again
        shutdown_phrases = (['Dobby has no master. Dobby is free!',
                             'Look at me. I am the captain now.',
                             'After all these years, my time has come...'])

        # sleep makes sure agent gets stop command prior before shutting down
        rospy.loginfo(random.choice(shutdown_phrases))
        rospy.sleep(1)
