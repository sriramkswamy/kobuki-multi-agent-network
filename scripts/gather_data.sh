#!/usr/bin/env bash

REMOTE_NAME="indigo"

# set the relevant folders
LOCAL_BASE="$HOME/Dropbox/Lab/Turtlebot"
REMOTE_BASE="~/catkin_ws/src"

PACKAGE_FOLDER="multi_agent"
REMOTE_FOLDER=$PACKAGE_FOLDER/scripts/data
LOCAL_FOLDER=$1/

# copy the data from the remote computer to the argument directory
mkdir -p $LOCAL_FOLDER
scp -r $REMOTE_NAME:$REMOTE_BASE/$REMOTE_FOLDER/* $LOCAL_FOLDER/
