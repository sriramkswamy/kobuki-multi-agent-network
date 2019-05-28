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

import agent
import rospy

test_agent = agent.Turtlebot(INPUT_FOLDER + '/simple_talk.json')

step = 1
while not rospy.is_shutdown():
    rospy.loginfo('Time step: ' + str(step))
    if step <= test_agent.time_steps:
        test_agent.publish()
        step += 1
    else:
        test_agent.shutdown()
