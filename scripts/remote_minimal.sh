#!/usr/bin/env bash

REMOTE_NAME="indigo"

# set all necessary folders
LOCAL_BASE="$HOME/Dropbox/Lab/Turtlebot"
REMOTE_BASE="~/catkin_ws/src"

PACKAGE_FOLDER="multi_agent"
SCRIPTS_FOLDER=$PACKAGE_FOLDER/scripts
SOURCE_FOLDER=$PACKAGE_FOLDER/scripts/rosagent
EXE_FOLDER=$PACKAGE_FOLDER/scripts/exe
LAUNCH_FOLDER=$PACKAGE_FOLDER/launch
ARGS_FOLDER=$PACKAGE_FOLDER/scripts/args
INPUT_FOLDER=$PACKAGE_FOLDER/scripts/input

GENERATE_SCRIPT="generate.py"

# generate the required files
echo "Generating inputs and launch files ..."
python $LOCAL_BASE/$SCRIPTS_FOLDER/$GENERATE_SCRIPT

# copy all relevant folders
echo "Copying folders to remote base station ..."
scp -r $LOCAL_BASE/$LAUNCH_FOLDER/* $REMOTE_NAME:$REMOTE_BASE/$LAUNCH_FOLDER/
scp -r $LOCAL_BASE/$SOURCE_FOLDER/* $REMOTE_NAME:$REMOTE_BASE/$SOURCE_FOLDER/
scp -r $LOCAL_BASE/$EXE_FOLDER/* $REMOTE_NAME:$REMOTE_BASE/$EXE_FOLDER/
scp -r $LOCAL_BASE/$ARGS_FOLDER/* $REMOTE_NAME:$REMOTE_BASE/$ARGS_FOLDER/
scp -r $LOCAL_BASE/$INPUT_FOLDER/* $REMOTE_NAME:$REMOTE_BASE/$INPUT_FOLDER/

echo "Quick base station setup complete."

