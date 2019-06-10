This file contains the set of all commands to do one multi-agent test
=====================================================================

Context
-------

Where to run the said commands:

- {Local} refers to the command line (terminal) in the local machine. You can get here by opening PowerShell/WSL for Windows and Terminal for Mac.
- {Base} refers to the command line (after ssh) in the base station. You can get here by doing: `ssh indigo`
- {Bot} refers to the command line (after ssh) in the Turtlebot. You can get here by doing: `ssh violet`, `ssh blue`, etc.

What to replace for each placeholder within a command:

- `<color>` refers to red, blue, green, violet, orange, or yellow.
- `<local_multi_agent_folder>` refers to the top level folder called `multi_agent` within the local computer. This folder contains `CMakeLists.txt`, `package.xml`, `include` folder, `launch` folder, `src` folder, and `scripts` folder.
- `<remote_multi_agent_folder>` refers to the top level `multi_agent` folder in the base station or the bot located at `~/catkin_ws/src/multi_agent`
- `<remote_multi_agent_scripts_folder>` refers to the `scripts` folder in the base station or the bot located at `~/catkin_ws/src/multi_agent/scripts`
- `<remote_multi_agent_data_folder>` refers to the `data` folder in the base station or the bot located at `~/catkin_ws/src/multi_agent/data`

Pre-processing
--------------

First, we need to design a configuration file in our local folder

1. {Local}: Go to the `<local_multi_agent_folder>`
	- This can be done by using the `cd` command from the terminal
	- If already in VS Code, ensure that you are in the right folder by using the `pwd` command to check the complete path of your current location
2. {Local}: Create an argument file within `<local_multi_agent_folder>/scripts/args/trials/`. For this set of instructions, there are already argument files created here. You can also use `expts` or `trials` directory instead of the `tests` directory.
3. {Local}: `scp <local_multi_agent_folder>/scripts/args/trials/test_rainbow_forward.json indigo:<remote_multi_agent_scripts_folder>/args/trials/`. Replace the filenames and folder names appropriately if changed.

Do the following in the base station:

1. {Base}: `ros_setup`
2. {Base}: `ros_master_<color>`
3. {Base}: `roscd multi_agent`
4. {Base}: `python scripts/generate.py`
	- To run this you should be in the `<remote_multi_agent_folder>` folder
	- You can check that by using `pwd`
	- If not, change to the folder by running `roscd multi_agent` or `cd ~/catkin_ws/src/multi_agent`

For each Turtlebot involved in the experiment do the following after making sure it is connected to the "Kumar ARC" network.

1. {Bot}: `ros_setup`
2. {Bot}: `ros_master_<color>`
3. {Bot}: `roslaunch multi_agent multi_minimal.launch`
4. {Bot}: `roslaunch multi_agent multi_3dsensor.launch`

Running the start calibration
-----------------------------

This section assumes you have run all the commands in the previous section or have done some equivalent of it.

1. {Base}: `roscd multi_agent/scripts` or `cd ~/catkin_ws/src/multi_agent/scripts`
2. {Base}: `rm -r data/*`
3. {Base}: `roslaunch multi_agent test_rainbow_stationary.launch`
4. {Local}: `cd <local_multi_agent_folder>`
	- If you want to check whether you are already in the folder, you can use `pwd` command and check its output
	- If you are somewhere relative to the path, use the appropriate `cd` commands to get here
5. {Local}: `cd scripts`
6. {Local}: `mkdir -p data/trials/sample_trial01/calibration/start`
	- This assumes the name of this trial is `sample_trial01` but give trials more descriptive names like `orange_red_straight_line_trial01`
	- You can change the name of `sample_trial01` to anything you want as long as there are no spaces in the name
7. {Local}: `scp -r indigo:~/catkin_ws/src/multi_agent/scripts/data/* data/trials/sample_trial01/calibration/start/`
8. {Local}: Note the offset from this in the appropriate row in the `trials.md` file under the `measurements` folder

Running the actual experiment
-----------------------------

Before you start this, rearrange the bots and run the {Bot} setup from the Pre-processing section once more.

1. {Base}: `roscd multi_agent/scripts` or `cd ~/catkin_ws/src/multi_agent/scripts`
2. {Base}: `rm -r data/*`
3. {Base}: `roslaunch multi_agent test_rainbow_forward.launch`
	- Note the launch file used here is the one created in the Pre-processing section
	- If this is not the same one you are using, change the name appropriately
4. {Local}: `cd <local_multi_agent_folder>`
	- If you want to check whether you are already in the folder, you can use `pwd` command and check its output
	- If you are somewhere relative to the path, use the appropriate `cd` commands to get here
5. {Local}: `cd scripts`
6. {Local}: `scp -r indigo:~/catkin_ws/src/multi_agent/scripts/data/* data/trials/sample_trial01/`
	- This assumes the name of this trial is `sample_trial01` but give trials more descriptive names like `orange_red_straight_line_trial01`
	- You can change the name of `sample_trial01` to anything you want as long as there are no spaces in the name
7. {Local}: Note the initial position, final position of each target and the time taken for simulation in the `trials.md` file under the `measurements` folder

Running the end calibration
---------------------------

Before you start this, rearrange the bots and run the {Bot} setup from the Pre-processing section once more.

1. {Base}: `roscd multi_agent/scripts` or `cd ~/catkin_ws/src/multi_agent/scripts`
2. {Base}: `rm -r data/*`
3. {Base}: `roslaunch multi_agent test_rainbow_stationary.launch`
4. {Local}: `cd <local_multi_agent_folder>`
	- If you want to check whether you are already in the folder, you can use `pwd` command and check its output
	- If you are somewhere relative to the path, use the appropriate `cd` commands to get here
5. {Local}: `cd scripts`
6. {Local}: `mkdir -p data/trials/sample_trial01/calibration/end`
	- This assumes the name of this trial is `sample_trial01` but give trials more descriptive names like `orange_red_straight_line_trial01`
	- You can change the name of `sample_trial01` to anything you want as long as there are no spaces in the name
7. {Local}: `scp -r indigo:~/catkin_ws/src/multi_agent/scripts/data/* data/trials/sample_trial01/calibration/end/`
8. {Local}: Note the offset from this in the appropriate row in the `trials.md` file under the `measurements` folder

Post-processing
---------------

This assumes all data is available in `<local_multi_agent_folder>/scripts/data/trials/sample_trial01`

1. {Local}: Go to the `<local_multi_agent_folder>`
2. {Local}: Go to the `constants` sub-folder within the `scripts` folder
3. {Local}: Open `constants.py`
	- Create an entry in the `TRIALS_DATA_FOLDER` with the key name same as the data folder name: `sample_trial01` in this case
	- Create an entry in the `TRIALS_PLOTS_FOLDER` with the key name same as the data folder name: `sample_trial01` in this case
4. {Local}: Open `trials_combos.py`
	- Create an entry named `sample_trial01` with the all similar details as the one provided in the template
	- Fill in the initial and final cartesian positions based on measurements noted in the `trials.md` file
5. {Local}: Open `process.py` in the `<local_multi_agent_folder>/scripts` folder
	- Change the argument in line 103 inside the `process_single_sim` function call
	- This value should be `sample_trial01` or any other value that has been consistently changed in all the previous steps
6. {Local}: Navigate to the `<local_multi_agent_folder>`
	- If you want to check whether you are already in the folder, you can use `pwd` command and check its output
	- If you are somewhere relative to the path, use the appropriate `cd` commands to get here
7. {Local}: `mkdir -p scripts/plots/trials/sample_trial01`
8. {Local}: `python scripts/process.py trials`
	- The argument for the scripts file is `trials` in this case indicating that ALL the data is stored within sub-folders of the `trials` folder
	- Change this as per needs. Common alternatives are `tests` or `expts` to specify that the data is stored within those folders
