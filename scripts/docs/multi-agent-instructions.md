Instructions for experiments and tests
======================================

Pre-processing
--------------

### Setup

This setup process should be completed when any new terminal is started

In the turtlebot:

1. In the terminal, run `ros_setup`. This sets up the unique name, IP, and hostname of the robot. It also runs `catkin_make` and puts you in `catkin_ws`. Note that this is not a standard ROS command. This is a user-defined function to keep things simple.
2. Setup the master for each robot using the user-defined function - `ros_master_<bot_name>`
3. Turn on the base, ensure the network is connected properly, and launch the multi agent minimal launch file like so: `roslaunch multi_agent multi_minimal.launch`
4. If the laser scanner is required, open a new terminal, repeat steps 1 and 2, then launch the 3D sensor like so: `roslaunch multi_agent multi_3dsensor.launch`
5. You need to use one system other than the base station for data transfer and post processing. Create a folder in your local system for this purpose and organize it based on the naming convention followed in the tests file.

### Establishing the global reference frame (calibration)

Each bot should be calibrated before _and_ after each experiment for **each** turtlebot

1. Navigate to the multi agent scripts folder after setup like so: `roscd multi_agent/scripts`
2. Clear the pre-existing data in the data folder like so: `rm -r data/*`
3. A slender pole is used for understanding the offsets in position and orientation
4. Place the pole within the field of view straight ahead such that the approximate angle is 0 radians
5. Measure the distance from the center of the turtlebot to the pole manually (using an inch/meter tape). _Note_: It's always better to do it in meters since the robot uses SI units.
6. From the base station (or after ssh-ing in to the base station) run the following command from the scripts directory: `roslaunch multi_agent test_rainbow_stationary.launch`. This script "moves" the bot by 0 distance.
7. Navigate to the local folder setup in step 5 of the setup section like so: `cd <local_folder_path>`. This will work in Windows PowerShell as well as Mac Terminal.
8. Transfer the data to your local computer. First, create a top level directory called `trials` (or any other name) that is going to store the raw data for each simulation in sub folders. Let's assume you have create one such sub-folder called `<awesome_simulation_trial1>`
	- Navigate to the `<awesome_simulation_trial1>` where you want to place **all** the raw data for every trial
	- If you have not created this folder in your Finder/Explorer, you can run `mkdir -p <awesome_simulation_trial1>/calibration/start` on Terminal/PowerShell to create this folder
	- Copy the raw data from the base station to the local computer: `scp -r indigo:~/catkin_ws/src/multi_agent/scripts/data/* <awesome_simulation_trial1>/calibration/start/`. This command will work both on the Windows PowerShell as well as the Mac Terminal.
9. If there are multiple turtlebots to be calibrated, you can do it individually and transfer the data for them in separate folders or you can place the pole in such a way that it is within the field of view of every bot and use that data. _Note_: Do **not** randomly orient the bots to achieve the second option. You should orient all of them in the global x or global y direction.

### Input arguments

To create any new experiment/test, you first need to create the input files with relevant arguments. The input arguments is a single `json` file containing all parameters required for multi agent execution. You can learn the basics of JSON [here](https://www.w3schools.com/js/js_json_objects.asp "W3Schools JSON introduction") (you can ignore the JavaScript syntax in the link).

The goal here is to ultimately create launch files that initiate the appropriate number of nodes with the correct names and commands. [Each node type (executable) can take arguments](https://wiki.ros.org/roslaunch/XML/node "node xml arguments"). The input arguments are provided in a nested JSON file located in the appropriate folder within `multi_agent/scripts/args`. Let us call the sample input file `sample.json`. When this `sample.json` is provided as arguments for the `generate.py` script it does the following:

- Create smaller, specific JSON files to do one thing and one thing only, i.e., it creates smaller argument files to either publish or subscribe but not both.
- Creates a launch file named `sample.launch` in the `multi_agent/launch` folder that supplies the smaller file generated in the previous step as an argument
- The main file `multi_agent/scripts/rosagent/agent.py` parses this smaller JSON file and executes the commands specified in the arguments

For practical purposes, the input file `sample.json` can be treated as a big file where each top level key contains the smaller argument file supplied to the launch file. Therefore, the `sample.json` file has the following key-value pairs:

1. The top level key(s) is (are) the name(s) of file(s) to be generated for the low level arguments to the nodes in the launch file created when the `generate.py` script is called
2. Withing this top level key, there is another nested JSON object
3. The `init_type` contains the kind of initialization type used for creating a node in the `agent.py` file. For now, there is provision only for a simple initialization (`_simple_init`) where each node does exactly one thing - publishing or subscribing. In the future, there is a possibility of creating a complex initialization (`_complex_init`) where each node contains publishers and subscribers.
4. `node_space` specifies the namespace of the node. This string is concatenated with the `node_name` string to create a unique node name when initializing a new node using the `rospy.init_node()` function in python.
5. `agent_name` refers to the name of the agent. This can be different from the `node_space` but is kept the same for now.
6. `agent_type` refers to the type of agent - Turtlebot in this case. This can be expanded for other types of agents in the future if required.
7. `time_steps` refers to the number of time steps required to publish/subscribe
8. `anonymous_node` indicates whether the node initiated with this JSON object must be anonymous or not
9. `comm_type` denotes whether the node to be initiated contains a Publisher or a Subscriber. Note that there are different types of publishers and subscribers which is indicated by the `node_type` key. The list of these executables can be found in the `multi_agent/scripts/exe/` directory.
10. The `subscribe` and `publish` keys contain data required for subscription and publication respectively. Most of the keys within these are self-explanatory once the [ROS beginner tutorials](https://wiki.ros.org/ROS/Tutorials "ros wiki link") are done.
11. The `namespace` key within the `publish` (or `subscribe`) key contains an additional namespacing layer if required.
12. The `comm_data` variable contains the name of the method within the `agent.py` class that has to be called to utilize the arguments. In case of `publish`, it also contains the linear and/or angular velocities if required.

### Coding on a local machine (recommended)

1. Create the new argument file on the local machine describing the test/experiment
2. Copy the argument file to the appropriate folder (`multi_agent/scripts/args`) on the base station using `scp`
3. Copy any other additional file changed to the base station too. If multiple people are working on the same code base, make sure to have a backup copy on the local machine and try not to cause conflicts on the base station.
4. `ssh` into the base station and navigate to the `multi_agent` folder. Run the generate script like so: `python scripts/generate.py`
5. This will generate the launch files required. Now remove the previous data using `rm -r scripts/data/*`
6. Navigate to the `multi_agent/scripts` folder and launch this file via `roslaunch multi_agent <name_of_the_arg_file_without_extension>.launch`. Tip: Have two ssh connections open to the base station, one in the scripts directory, and one in the multi_agent directory to avoid changing directories back and forth. These will remember the history commands in the correct order as well.
7. Transfer this data to the local machine similar to the step 8 in calibration. Note: Do not include the `calibration/start` folder here. Transfer it directly to the `<local_folder>`
8. Run the `process.py` script locally as shown in the Post-processing section further down this file.

_Note_: If you are on a Mac, there are 3 helpful scripts to do these steps in the local `multi_agent/scripts` directory:
- `remote_minimal.sh` will copy the relevant arguments and changed files from the local directory to the base station. You can run it like so: `./remote_minimal.sh`. Note: If it doesn't run, make it an executable first by using `chmod +x ./remote_minimal.sh`
- `gather_calibration.sh` will copy the calibration data to the local computer. This takes one argument - the path of the local folder where the data is to be copied. You can run it like so: `./gather_calibration.sh my_data_folder/test01`
- `gather_data.sh` will copy the raw data to the local computer. This also takes the same argument as the `./gather_calibration.sh` script

### Coding on the base station

1. Navigate to the `multi_agent` folder. Run the generate script like so: `python generate.py <name_of_the_arg_file_without_extension>`
2. This will generate the launch files required. Now remove the previous data using `rm -r scripts/data/*`
3. Navigate to the `multi_agent/scripts` folder and launch this file via `roslaunch multi_agent <name_of_the_arg_file_without_extension>.launch`.

Test/Experiment
---------------

For each individual test/experiment, do the following:

- Calibrate the observer w.r.t the global reference frame (if moved from first calibration)
- Place the target in the desired location oriented towards the global x or global y axis
- Calibrate the target w.r.t the global reference frame
- Calibrate the target w.r.t the global reference frame in this new location and orientation (if the orientation changed from the previous calibration)
- Conduct the experiment
- Calibrate the target w.r.t the global reference frame in the end position
- Repeat each test/experiment at least twice, i.e., get two trials to verify if results are reproducible

Post-processing
---------------

The pre-requisite is to install python packages. Use [Anaconda distribution for best results](https://www.anaconda.com/distribution/ "anaconda download link")

1. Open `multi_agent/scripts/constants/constant.py`
	1. `TRIALS_BASE_PLOTS_FOLDER` points to the top level directory where all plots are to be saved.
	2. `TRIALS_BASE_DATA_FOLDER` contains the top level folder that has **all** the data for tests/experiments/trials
	3. Create 2 [dictionaries](https://www.tutorialspoint.com/python/python_dictionary.htm "py dict") which maps the location of the directories and files that contain raw data.
		- `TRIALS_PLOTS_FOLDER` contains the map to all the individual directories where trials/tests/experiments plots are stored
		- `TRIALS_DATA_FOLDER` contains the map to all the individual directories where trials/tests/experiments data are stored
	4. Create a dictionary called `BOT_NAMES` if it does not exist already. This contains a map of bot names.
	5. Create a dictionary called `SCAN_LOCATION` if it does not exist already. This contains a map of folders containing the point cloud data
2. Create a new file (if it doesn't exist) called `trials_combos.py`
	1. Within this file you have to create a map of the information needed for _each_ combination
	2. The top level key in the nested dictionary stands for the folder name. This is the name of the folder where the data is stored. This is also the same as the top level keys in `TRIALS_DATA_FOLDER` and `TRIALS_PLOTS_FOLDER`.
	3. This top level dictionary has a few arguments:
        - `integration`: contains a boolean value whether an integration check has to be performed for velocities or not
        - `polar`: refers to whether all the files have information in the polar coordinates
        - `wall_gating_dist`: the distance after which all data points from laser scanner corresponds to the wall
        - `time`: time taken for the experiment measured manually using a stop clock
        - `type`: the type of simulation
        - `odom_positions`: a [list](https://www.tutorialspoint.com/python/python_lists.htm "py list") of variables that contain the file names of odometry positions for each target bot
        - `odom_velocities`: a list of variables that points to the velocity files of each target bot. Note that this list must be in same order as the list specified in the `odom_positions` variable.
        - `odom_dcm`: a list of variables that points to the directional cosine matrix files of each target bot. Note that this list must be in same order as the list specified in the `odom_positions` variable.
        - `odom_timestamps`: a list of variables that points to the odometer time stamps files of each target bot. Note that this list must be in same order as the list specified in the `odom_positions` variable.
        - `odom_quaternions`: a list of variables that points to the quaternions files of each target bot. Note that this list must be in same order as the list specified in the `odom_positions` variable.
        - `scan_values`: a variable that points to the complete laser scan data of the observer
        - `scan_timestamps`: a variable that points to the laser scan time stamps of the observer
        - `target_offset_polar`: a list of list that contains the range and angle values of offset for each target. Note that this list must be in same order as the list specified in the `odom_positions` variable.
        - `target_offset_cartesian`: a list of list that contains the x and y values of offset for each target. Note that this list must be in same order as the list specified in the `odom_positions` variable.
        - `observer_offset_polar`: a list of list that contains the range and angle values of offset for the observer. Note that this list must be in same order as the list specified in the `scan_values` variable.
        - `observer_offset_cartesian`: a list of list that contains the x and y values of offset for the observer. Note that this list must be in same order as the list specified in the `scan_values` variable.
        - `init_cartesian`: a list of list that contains the x and y values of the initial position for each target. Note that this list must be in same order as the list specified in the `odom_positions` variable. This is measured manually.
        - `init_polar`: a list of list that contains the range and angle values of final position for each target. Note that this list must be in same order as the list specified in the `odom_positions` variable. This is measured manually.
        - `fin_cartesian`: a list of list that contains the x and y values of the initial position for each target. Note that this list must be in same order as the list specified in the `odom_positions` variable. This is measured manually.
        - `fin_polar`: a list of list that contains the range and angle values of final position for each target. Note that this list must be in same order as the list specified in the `odom_positions` variable. This is measured manually.
4. Finally, in the `process.py` file within the `multi_agent/scripts` directory, specify the name of the trial (same as the top level keys in `trials_combos.py`, `TRIALS_PLOTS_FOLDER`, and `TRIALS_DATA_FOLDER`) as an argument to the `process_single_sim` function. The second argument is the name of the folder containing all data - `trials` in this case.
5. Now, navigate to the `multi_agent` folder and run `python scripts/process.py trials` and this should place all the plots within the folder specified in `TRIALS_PLOTS_FOLDER`
