Instructions for single agent tests
===================================

Pre-processing
--------------

1. Check if the network is setup correctly. For each new terminal run, `ros_setup` first and then `ros_master_<bot_name>`.
2. Run `roslaunch turtlebot_bringup minimal.launch --screen` in a terminal on the bot
	- This launches the minimal configuration required for moving the bot
	- The screen option indicates that this command will not be shutdown even if the laptop screen is closed
3. Run `roslaunch turtlebot_bringup 3dsensor.launch --screen` in a terminal on the bot if the laser scanner is required
4. Run `ros_master_<bot_name>` in a terminal on the base station
5. In the python file, the first line must be `#! /usr/bin/env/ python`
6. Navigate to python script folder and run `chmod +x script_name.py`. This makes the script executable

Test/Experiment
---------------

### Coding on the base station using VS Code

1. Open VS Code on the local computer
2. Navigate the the "Remote-SSH" sidebar indicated by a computer symbol
3. Connect to the base station (`indigo`) by hovering over its name and opening in a new window
4. Make the necessary changes and save the file
5. Open the terminal inside VS Code by typing the shortcut `Ctrl+Shift+P` (`Cmd+Shift+P` if on a Mac) and typing "New terminal". Choose the first option.
6. Launch the appropriate script one of two ways from this terminal:
	- Navigate to the python script folder and run `python script_name.py` on the base station in the same terminal
	- Navigate to python script folder and run `roslaunch <ros_package_name> script_name.py` on the base station in the same terminal
	- If the first method works, but the second does not, then there is some problem in the ros setup. Use `ros_setup` command first and redo these steps
7. To debug any file, there needs to be a configuration created in `launch.json` file. There are already examples setup for most things. Copy, paste, and modify if necessary

_Note_: This method might not always work because, as of now, VS Code expects an internet connection on both the local and remote machines. Since the base station needs to be connected to "Kumar ARC" network, there is no internet connection.

### Coding on the local machine using any editor (prefer VS Code)

1. Check for any errors on the local system if possible
2. Open a new terminal (or an intergrated terminal in VS Code like in step 5 in the previous section)
3. Secure copy the current file to the appropriate location on the remote machine like so - `scp <current_file> <remote_name>:~/catkin_ws/src/<ros_package_name>/scripts/<current_file>`. An example would be `scp goforward.py indigo:~/catkin_ws/src/basic_tests/scripts/goforward.py`
4. SSH into the remote machine like so - `ssh <remote_name>`. E.g. `ssh indigo`
5. Launch the appropriate script one of two ways from this terminal:
	- Navigate to the python script folder and run `python script_name.py` on the base station in the same terminal
	- Navigate to python script folder and run `roslaunch <ros_package_name> script_name.py` on the base station in the same terminal
	- If the first method works, but the second does not, then there is some problem in the ros setup. Use `ros_setup` command first and redo these steps
6. To debug any file, you would have to look at the error message after step 5, change code in the local machine and follow steps 1 to 4 again

Post-processing
---------------

1. Save data from odometry from all time steps using `numpy` or `write` (from the built-in library)
2. Save the laser scan data in 2 ways:
	- Save the complete closest point data in a single file
	- Create a folder to save all the point cloud data from various time steps
3. Transform the coordinates based on calibration
4. Plot using `matplotlib.pyplot`
