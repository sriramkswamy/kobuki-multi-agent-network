Troubleshooting Tips and Tricks
===============================

Sanity checks
-------------

### Network checks

1. All the agents and base station are connected to the same network - `Kumar ARC` in this case. If not, ensure that they are connected to this network. Sometimes, a restart might be required
2. The result of the IP address from `ifconfig` matches the environment variables `ROS_IP` and `ROS_HOSTNAME`
	- Open a new terminal and run `ifconfig`
	- Now run `echo $ROS_HOSTNAME` and `echo $ROS_IP`
	- If they are not the same, set these variables to be the same via `export ROS_IP=<ifconfig IP>` and `export HOST_NAME=<ifconfig IP>`
	- Make the same changes in the ROS configuration file located at `~/.rosrc`
3. The result of environment variable `ROS_MASTER_URI` should be `http://<master IP>:11311` unless the port was manually changed in the ROS configuration. If this is not the case, make the necessary changes similar to the procedure described in the previous step
4. The topics and nodes seen by all agents via the command `rostopic list` and `rosnode list` respectively must be the same.

### Data checks

1. The quaternions received from the odometer measurements must be normalized

Setup errors
------------

### Installation issues

1. Kobuki Device not found: There can be multiple reasons for this
	- Check if the cables from the base to the laptop are attached properly.
	- Check if the base is charged. The green LED should be on when the base is turned on.
	- Check if the `udev` is setup properly. Follow the instructions [here](https://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/TurtleBot%20Bringup "udev test link").
	- Check if the `kinectdriver` is installed correctly. Follow there instructions [here](http://learn.turtlebot.com/2015/02/01/8/ "kinect driver installation link"). _Note_: For the kinect test command use `roslaunch freenect_launch freenect.launch` instead of `openni`.

### Networking issues

1. Unable to connect to "master"
	- Sometimes bots/workstations disconnect from network. Run it again first. Else, follow the proceeding steps
	- Check if all the network cards are up and connected to Kumar ARC. Disconnect it and connect again if necessary.
	- Check if the `ROS_MASTER_URI` is correctly setup for each bot by using `echo $ROS_MASTER_URI`. If not, set it up by using `export ROS_MASTER_URI='http://<master_ip>:11311'`
	- Check if the commands `ros_master_<bot_name>` points to the correct IP for each bot. To do this, open `~/.rosrc` (`gedit ~/.rosrc`) on each bot and verify the IPs listed at the top are the same. This verification can be done by using `ifconfig` in each bot.
2. Unable to connect to my own server (or similar)
	- Sometimes bots/workstations disconnect from network. Run it again first. Else, follow the proceeding steps
	- Check if `ROS_IP`, `ROS_HOSTNAME`, and `ROS_MASTER_URI` have the same IP address using `echo $ROS_HOSTNAME` and/or `echo $ROS_IP` and/or `echo $ROS_MASTER_URI`
	- Check if this IP address is the same as the one from `ifconfig`
	- If not, open a terminal and type `gedit ~/.rosrc` and ensure all of these things match
	- Run `ros_setup` again
3. `ssh no route to host` or any other ssh error
	- Check if your ssh config (`~/.ssh/config`) if all the `Host`, `HostName`, and `User` data match the `ifconfig` and login details of the respective bots/workstations
	- To open this, VS Code has an easy way. Open VS Code Insiders. Click on the sidebar "Remote-SSH" extension and click on the gear icon that appears when hover over the "Connections" dropdown menu. Open the first file from the options given and make the appropriate changes.
4. Other issues when launching/copying/`ssh`-ing into a machine
	- These network problems are hard to deal with. If you are at this stage, there is no clear solution. You have to run some prognostics.
	- Open the terminal in your computer and try `ssh`-ing in to each machine
	- If you are not able to `ssh` into machine, see if you can `ping` it like so: `ping 10.0.1.19`. Change the IP to the appropriate IP of the remote machine.
	- If you are unable to ping either, this means that this access point is not functioning. This could be an internal driver issue or an external WiFi card issue. Try restarting the machine. If that doesn't work, try changing the WiFi card (_Note_: This will change the IP of that machine). This typically would resolve this issue.
	- If you are able to `ping` but not `ssh`, the issue becomes hard to diagnose. Only [Google](https://www.google.com/search?hl=en&q=able%20to%20ping%20but%20not%20ssh "search link") can save you now.

Runtime errors
--------------

