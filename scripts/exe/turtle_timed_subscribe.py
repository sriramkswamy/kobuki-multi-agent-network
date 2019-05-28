#!/usr/bin/env python

import os
import sys
import inspect

# realpath() will make your script run, even if you symlink it

# folder containing python agent module
MODULE_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
MODULE_FOLDER = os.path.join(MODULE_FOLDER, 'rosagent')

# folder containing the json inputs
INPUT_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
INPUT_FOLDER = os.path.join(INPUT_FOLDER, 'input', '')

# folder containing the output data
DATA_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
DATA_FOLDER = os.path.join(DATA_FOLDER, 'data', '')

# Add path for agent module
if MODULE_FOLDER not in sys.path:
    sys.path.insert(0, MODULE_FOLDER)

import agent
import rospy

test_agent = agent.Turtlebot(INPUT_FOLDER + sys.argv[1], DATA_FOLDER)

step = 1
while not rospy.is_shutdown():
    rospy.loginfo('Time step: ' + str(step))
    if step <= test_agent.time_steps:
        test_agent.subscribe()
        step += 1
    else:
        test_agent.shutdown()
