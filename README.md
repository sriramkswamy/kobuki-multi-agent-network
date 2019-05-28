ROS Multi agent package for data collection
===========================================

This is a ROS package for collecting laser scan data (and point cloud data if necessary) from multiple [Kobuki Turtlebots](https://www.turtlebot.com/turtlebot2/ "Turtlebot 2 link"). There are six Turtlebots here, each named after a color of the rainbow (except indigo which is the ground station).

Installation
------------

1. Put this inside catkin workspace source directory (typically `~/catkin_ws/src`)
2. Run `catkin_make` from the workspace directory (`~/catkin_ws` if using the previous convention)
3. `roscd multi_agent/scripts` and launch any multi agent pattern using `roslaunch`
4. Further documentation present in `scripts/docs`

FAQs
----

### What is this code used for?

It is used to move up to 6 Kobuki bots with specified linear and angular velocities. There is no autonomous element here. This is used for data collection for testing a [tensor decomposition based framework for data association](https://arc.aiaa.org/doi/abs/10.2514/6.2018-1134).

### Why create a bunch of small input files to create multiple nodes? Is it necessary?

I wanted it make it such that each node does one thing and one thing only. So, there is some problem when publishing data from one of the Turtlebots, the subscriber does not shutdown too. Also, I did not need to share data during the simulations. I just needed to store the data from each sensor. Therefore, no data sharing is required between nodes. In case there is some data that needs to be shared (filtering online and path planning for example), it is better to use a client/server architecture.

### Why not use service/client architecture in ROS?

The goal was to communicate to each bots individually and make them move in a "known" pattern which will be used as the process model (with noise) for filtering and association. This data will be used for data association in tracking problems offline and there is no online autonomous navigation for now. Of course, if this changes in the future, service/client architecture is the way to go. Since I wanted to communicate with each bot separately though they are in a multi agent network, I chose the publish/subscribe architecture. It may not be the best choice for everything but it seemed to fit my needs.

### Why create a monolith agent class that handles everything? Why not separate publishers/subscribers for specific things?

This is a good question; I do not have a great answer for this. The idea was to supply only as many arguments as needed and make it simple for me to run multiple simulations quickly. Plus, this code was intended to be expandable to drones in the future. That's why the class architecture. In hindsight, this might not have been the best idea but I kind of like the fact that I limit my executable files and keep them generic while handling all bot related functions within its own class. I will probably redesign this if I were to do it again.

### Why the large constants folder?

Honestly, I over-engineered the post-processing and locked myself in. This was not a great idea.
