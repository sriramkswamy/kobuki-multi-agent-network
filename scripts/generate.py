#!/usr/bin/env python

import os
import sys
import inspect
import json
import xml.etree.ElementTree as et

# realpath() will make your script run, even if you symlink it

# folder containing python agent module
MODULE_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
MODULE_FOLDER = os.path.join(MODULE_FOLDER, 'scripts', 'rosagent')

# Add path for agent module
if MODULE_FOLDER not in sys.path:
    sys.path.insert(0, MODULE_FOLDER)

print MODULE_FOLDER

import agent
import rospy


def create_jsons(data_file):
    """Breaks down the input file into simpler json files and sets node data"""
    # load all the data
    with open(data_file) as json_file:
        input_data = json.load(json_file)

    # initialize node details for launch files
    node = {}
    node['names'] = []
    node['types'] = []
    node['args'] = []
    node['pkg'] = 'multi_agent'
    node['cwd'] = 'node'
    node['output'] = 'screen'
    node['num'] = 0

    # parse input
    for key in input_data:
        simple_data = input_data[key]

        # get data for nodes
        node['num'] += 1
        simple_name = simple_data['node_space'] + simple_data['node_name']
        node['names'].append(simple_name)
        simple_type = 'turtle_' + simple_data['node_type'] + '.py'
        node['types'].append(simple_type)
        # simple_file = INPUT_FOLDER + key + '.json'
        simple_file = key + '.json'
        node['args'].append(simple_file)

        # break down input into smaller files
        breakdown_file = INPUT_FOLDER + key + '.json'
        with open(breakdown_file, 'w') as outfile:
            json.dump(simple_data, outfile, indent=4, sort_keys=True)

    return node


def create_launch(launch_nodes, launch_file):
    """Creates the launch file given the nodes data"""
    launch_tag = et.Element('launch')
    num_nodes = launch_nodes['num']
    items = []

    count = 0
    while count < num_nodes:
        new_item = et.SubElement(launch_tag, 'node')
        new_item.set('name', launch_nodes['names'][count])
        new_item.set('type', launch_nodes['types'][count])
        new_item.set('args', launch_nodes['args'][count])
        new_item.set('pkg', launch_nodes['pkg'])
        new_item.set('cwd', launch_nodes['cwd'])
        new_item.set('output', launch_nodes['output'])
        items.append(new_item)
        count += 1

    launch_content = et.tostring(launch_tag)
    with open(launch_file, 'w') as outfile:
        outfile.write(launch_content)

    return launch_tag

# folder containing the json inputs
INPUT_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
INPUT_FOLDER = os.path.join(INPUT_FOLDER, 'scripts', 'input', '')

# folder containing the output data
DATA_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
DATA_FOLDER = os.path.join(DATA_FOLDER, 'scripts', 'data', '')

# folder containing the arguments
ARG_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
ARG_FOLDER = os.path.join(ARG_FOLDER, 'scripts', 'args', '')

# folder containing the launch files
LAUNCH_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
LAUNCH_FOLDER = os.path.join(LAUNCH_FOLDER, 'launch', '')

# arguments for tests and experiments
TESTS_ARGS = os.path.join(ARG_FOLDER, 'tests', '')
EXPTS_ARGS = os.path.join(ARG_FOLDER, 'expts', '')
TRIALS_ARGS = os.path.join(ARG_FOLDER, 'trials', '')

# final launch configuration folders
TESTS_LAUNCH = os.path.join(LAUNCH_FOLDER, 'tests', '')
EXPTS_LAUNCH = os.path.join(LAUNCH_FOLDER, 'expts', '')
TRIALS_LAUNCH = os.path.join(LAUNCH_FOLDER, 'trials', '')

if __name__ == "__main__":
    tests_path, tests_dirs, tests_files = next(os.walk(TESTS_ARGS))
    expts_path, expts_dirs, expts_files = next(os.walk(EXPTS_ARGS))
    trials_path, trials_dirs, trials_files = next(os.walk(TRIALS_ARGS))

    # generate launch files for all tests
    if tests_files:
        for json_file in tests_files:
            file_base_name = os.path.splitext(json_file)[0]
            launch_file = file_base_name + '.launch'
            tests_nodes = create_jsons(os.path.join(TESTS_ARGS, json_file))
            tests_launch = create_launch(tests_nodes,
                                         os.path.join(TESTS_LAUNCH, launch_file))
    else:
        print('No argument files found for tests')

    # generate launch files for all experiments
    if expts_files:
        for json_file in expts_files:
            file_base_name = os.path.splitext(json_file)[0]
            launch_file = file_base_name + '.launch'
            expts_nodes = create_jsons(os.path.join(EXPTS_ARGS, json_file))
            expts_launch = create_launch(expts_nodes,
                                         os.path.join(EXPTS_LAUNCH, launch_file))
    else:
        print('No argument files found for experiments')

    # generate launch files for all trials
    if trials_files:
        for json_file in trials_files:
            file_base_name = os.path.splitext(json_file)[0]
            launch_file = file_base_name + '.launch'
            trials_nodes = create_jsons(os.path.join(TRIALS_ARGS, json_file))
            trials_launch = create_launch(trials_nodes,
                                         os.path.join(TRIALS_LAUNCH, launch_file))
    else:
        print('No argument files found for trials')
