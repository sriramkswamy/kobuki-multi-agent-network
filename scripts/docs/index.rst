ROS Multi Turtlebot Simulation
==============================

Setup
-----

`Robot Operating System (ROS) <http://www.ros.org/about-ros/>`__ is a
general purpose framework for writing robot software. Since a
considerable amount of ROS terminology is utilized in the following
sections, it will be helpful to learn about these if necessary. The
following sections give a quick primer on ROS and can be skipped if it
is already known how ROS works.

Installation
~~~~~~~~~~~~

Before installing ROS on any machine, it is important to note the
supported operating systems for each version. Each `long term
support <https://wiki.ros.org/ROS/Installation/>`__ (LTS) version of ROS
is attached to a corresponding operating system version, typically
`Ubuntu LTS <https://wiki.ubuntu.com/LTS/>`__ . Furthermore, only Linux
is officially supported as a platform for installing ROS. Finally, each
robot has its own requirements on which ROS version is supported, e.g.,
`Turtlebot 2 <https://robots.ros.org/turtlebot/>`__ (used here) supports
only `ROS Indigo <https://wiki.ros.org/indigo/Installation/>`__ .
Therefore, for these simulations, only ROS Indigo is used in all the
related computers, i.e., each Turtlebot has a tweaked version of Ubuntu
14.04 provided along with the hardware. The base station also runs only
Ubuntu 14.04. It should be noted is it is *possible* to run different
versions of ROS on the Turtlebot and the local device as long as all the
code being used uses only features provided in ROS Indigo and no
additional ones.

Turtlebot
^^^^^^^^^

The provided installation drive for Ubuntu also has ROS installed for
the Turtlebot

Local Machine
^^^^^^^^^^^^^

First follow the `Ubuntu 14.04 install
instructions <https://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr/>`__
. Once the OS is installed and updated via ~sudo apt-get update && sudo
apt-get upgrade~. Finally, follow the instructions for installing `ROS
Indigo on Ubuntu <https://wiki.ros.org/indigo/Installation/Ubuntu/>`__ .
If installing a different version of Ubuntu and/or ROS, follow their
appropriate installation instructions.

Networking
~~~~~~~~~~

For multiple agents to communicate, they need to be on the same wireless
`AirPort <https://support.apple.com/airport/>`__ network. For these
simulations an without internet access was used as the router.

Master
^^^^^^

Node
^^^^

Publishing
^^^^^^^^^^

Subscribing
^^^^^^^^^^^

Services
^^^^^^^^

Clients
^^^^^^^

Turtlebot
~~~~~~~~~

`Tutorials <http:learn.turtlebot.com/>`__

Running the simulations
-----------------------

Turtlebot(s) and base station
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Supplying the arguments
~~~~~~~~~~~~~~~~~~~~~~~

Source
------

Organization
~~~~~~~~~~~~

Package
^^^^^^^

Launch files
^^^^^^^^^^^^

Script file execubtables
^^^^^^^^^^^^^^^^^^^^^^^^

Directory structure
~~~~~~~~~~~~~~~~~~~

Classes
~~~~~~~

Helper functions
~~~~~~~~~~~~~~~~

Useful scripts
~~~~~~~~~~~~~~

Troubleshooting
---------------

Sanity checks
~~~~~~~~~~~~~

Networking configuration
^^^^^^^^^^^^^^^^^^^^^^^^

Nodes and topics
^^^^^^^^^^^^^^^^

Debugging
~~~~~~~~~

Python debugger
^^^^^^^^^^^^^^^

VS Code debugger
^^^^^^^^^^^^^^^^

Additional resources
--------------------

Python
~~~~~~

C++
~~~

MATLAB
~~~~~~
