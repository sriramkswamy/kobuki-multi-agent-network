#!/usr/bin/env python

import os
import sys
import inspect

# realpath() will make your script run, even if you symlink it
# import the newly defined agent module
MODULE_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])) +
                 '/rosagent')
if MODULE_FOLDER not in sys.path:
    sys.path.insert(0, MODULE_FOLDER)

# realpath() will make your script run, even if you symlink it
# import the newly defined agent module
INPUT_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])) +
                '/tests/input')
if INPUT_FOLDER not in sys.path:
    sys.path.insert(0, INPUT_FOLDER)

# realpath() will make your script run, even if you symlink it
# import the newly defined agent module
OUTPUT_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])) +
                 '/data/')

import agent
import rospy

test_agent = agent.Turtlebot(INPUT_FOLDER + '/complex_comm_move.json')

test_agent.subscribe()
rospy.spin()
