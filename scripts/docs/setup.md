Turtlebot and ROS setup
=======================

Linux
-----

The primary way of controlling the Turtlebots is via the [Robot Operating System
(ROS)](http://www.ros.org/about-ros/). ROS is best supported on Linux based
operating system (OS) - specifically Ubuntu or Debian distributions. To choose
the correct distribution, the following variables have to be kept in mind:

- Version of ROS supported by the robot
- Type of OS supported by that version of ROS
- Any other hardware limitations/requirements for that version of the OS

Based on all this, the only OS applicable for all agents and workstations
involved here is the [Ubuntu
14.04](https://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr) long
term support ([LTS](https://wiki.ros.org/ROS/Installation/)) and the
corresponding ROS version is [ROS
Indigo](https://wiki.ros.org/indigo/Installation/).

_Note_: It is possible to install a higher version of Ubuntu or ROS in a local
system. However, in such a case, care must be taken that the code uses only
features/syntax supported by the Turtlebots. It is strongly suggested not to
fiddle around with the versions unless the person is aware of what they are
doing.

ROS
---

ROS is a general purpose framework for writing robot software.

Turtlebot
---------

[Tutorials](http:learn.turtlebot.com/)

### OS installation

### Kinect Driver

Base Station
------------

Networking
----------

For multiple agents to communicate, they need to be on the same wireless
network - [AirPort](https://support.apple.com/airport/) in this case. For these
simulations an without internet access was used as the router.

Equipment
---------

- Camera: Raise the tripod to its maximum height and position the front of the tripod at the red tape indicated
- Timer: Keep a timer and record the manual time required for each experiment
