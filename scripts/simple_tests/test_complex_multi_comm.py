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

try:
    test_agent = agent.Turtlebot(INPUT_FOLDER + '/complex_multi_comm.json')
    for step in xrange(test_agent.time_steps):
        rospy.loginfo('Time step: ' + str(step))
        test_agent.publish()
        test_agent.subscribe()
    test_agent.shutdown()
except (KeyboardInterrupt, SystemExit):
    test_agent.shutdown()
