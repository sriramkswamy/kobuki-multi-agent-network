Calibration
===========

This document discusses the various tests that can be done to see if the sensors
are working and/or calibrate them.

Pre-requisites:

- Turtlebot with a kinect driver installed
- Turtlebot minimal working condition
- Slender object for calibration (e.g. pole)
- ROS package `basic_tests` setup

Range sensor
------------

Some of the basic tests mentioned here are based on the [Turtlebot
tutorial](http://learn.turtlebot.com/2015/02/01/8/). Prior to the tests, ensure
the following setup is done:

- The `$TURTLEBOT_3D_SENSOR` environment variable is set to `kinect`. If the
  proper setup is followed, this should already be the case
- Open a new terminal
  - Run `ros_setup` which sets up the particular agent or ground station
  - Launch the minimal configuration for that Turtlebot `roslaunch turtlebot_bringup minimal.launch`
  - Launch the kinect sensor configuration `roslaunch freenect_launch freenect.launch`
- For any troubleshooting tips, follow the link mentioned above or check `troubleshooting.md`

### Image with RGB values

This is a test to check if the Kinect sensor is able to perceive 3D images.

- To see the image in a new window, run `rosrun image_view image_view image:=/camera/rgb/image_color`
- This should open a live stream from the Kinect camera in a new window
- The same image can also be visualized using `rviz` like so: `roslaunch turtlebot_rviz_launchers view_robot.launch`
- In the RViz panel, opening the panel "Image" from the list of panels will result in the same live stream

### Depth sensor information

This is a test to check if the Kinect sensor is able to perceive 3D depth sensor information.

- To see the image in a new window, run `rosrun image_view image_view image:=/camera/depth/image`
- This should open a live stream from the Kinect camera in a new window
- The same image can also be visualized using `rviz` like so: `roslaunch turtlebot_rviz_launchers view_robot.launch`
- In the RViz panel, opening the panel "Depth" from the list of panels will result in the same live stream

### Laser scanner

This test checks if the closest point in the laser scan is reported correctly.

- Place a slender object, a pole in this case, in a known location in the global reference frame
- After loading the `basic_tests` package run the `laserscan.py` source
- If the closest point reported is not approximately the same value as the calibration pole, note that bias down

Odometer
--------

### Origin position and orientation

This test checks if the odometer is reset after every reset of the Turtlebot.

- Place the Turtlebot at a known location on the map, and measure its reported odometer position. Both the reported position and orientation (Euler angles; not quaternions) must be zero at this location.
- Move the Turtlebot manually to a different known location on the map and record the odometer position. This position and orientation would be non-zero due to the fact that the Turtlebot has "moved" and also due to human error while moving the bot.
- Turn the Turtlebot off and on again in the same position. Record the new odometer readings and this should be reported as zero again.
- If there is an offset, include this offset in post processing

### Motion

This test checks if the control input matches the expected output of the odometer during motion.

- Place the Turtlebot at a known location on the map and provide input in only one direction. x input is interpreted as forward motion while y input is interpreted as sideward motion.
- After post-processing, the complete trajectory provided by the odometer must be a straight line in the direction of the provided input.
- If not, isolate each degree of freedom and identify the cause of the problem. Follow Troubleshooting tips described in `troubleshooting.md`

Single agent basic tests
------------------------

### Simple chatter

This is a test to check if ROS was installed correctly and the publisher/subscriber architecture is in place.

- Open a terminal in the agent and run `roscore`
- Open a new terminal and navigate to `~/catkin_ws/src/basic_tests/scripts`
- Run `python chatter.py`
- This should result in a series of info messages that spits of the test chatter phrase
- Press `CTRL+C` to end it

### Movement

This is a test to check if the agent is able to execute basic commands

- Open a terminal in the agent and run `roslaunch turtlebot_bringup minimal.launch`
- Open a new terminal and navigate to `~/catkin_ws/src/basic_tests/scripts`
- Run `python goforward.py`
- This agent should move forward with the given speed in the code
- Press `CTRL+C` to end it

### Measurement

This is a test to check if the agent is able to measure its position

- Open a terminal in the agent and run `roslaunch turtlebot_bringup minimal.launch`
- Open a new terminal and navigate to `~/catkin_ws/src/basic_tests/scripts`
- Run `python measure.py`
- This agent should report the odometer position and quaternions
- Moving the agent will also change these values
- Press `CTRL+C` to end it

### Laser scan

This is a test to check if the agent is able to scan other objects

- Open a terminal in the agent and run `roslaunch turtlebot_bringup minimal.launch`
- Open a terminal in the agent and run `roslaunch turtlebot_bringup 3dsensor.launch`
- Open a new terminal and navigate to `~/catkin_ws/src/basic_tests/scripts`
- Run `python laserscan.py`
- This agent should report the closest point position
- Moving an object closer than the current closest point should update this information
- Press `CTRL+C` to end it

Base station and networking test
--------------------------------

These tests check if the network is established between the agent and the base station.
- Run the commands that are meant to be run on agents for all tests described in the previous section
- Before running the python scripts, switch to the base station, open a new terminal and set the master to the desired agent
- Navigate to `~/catkin_ws/basic_tests/scripts` on the base station
- Perform all the same tests as above. The results should be exactly the same if the network is established correctly
- Check `troubleshooting.md` for any problems
