#!/usr/bin/env python

import os
import sys
import inspect

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

# folder containing python constants
CONSTANTS_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
CONSTANTS_FOLDER = os.path.join(CONSTANTS_FOLDER, 'scripts', 'constants')

# Add path for agent module
if CONSTANTS_FOLDER not in sys.path:
    sys.path.insert(0, CONSTANTS_FOLDER)

import transform_variables as tv
import perform_checks as pc
import parse_files as pf
import process_plots as pp
from expts_combos import *
from tests_combos import *
from constants import *


def process_single_sim(current_simulation, simulation_type):
    """plot and post process all information for a single simulation"""
    # details of the current simulation
    sim_type = simulation_type.upper()
    combos_name = sim_type + '_COMBOS'
    data_folder_name = sim_type + '_DATA_FOLDER'
    plots_folder_name = sim_type + '_PLOTS_FOLDER'
    sim_combos = getattr(sys.modules[__name__], combos_name)
    data_folder = getattr(sys.modules[__name__], data_folder_name)
    plots_folder = getattr(sys.modules[__name__], plots_folder_name)

    # set the current data
    current_data = {
        'data_folder': data_folder[current_simulation],
        'plots_folder': plots_folder[current_simulation],
        'odom_positions': sim_combos[current_simulation]['odom_positions'],
        'odom_velocities': sim_combos[current_simulation]['odom_velocities'],
        'odom_quaternions': sim_combos[current_simulation]['odom_quaternions'],
        'odom_dcm': sim_combos[current_simulation]['odom_dcm'],
        'odom_timestamps': sim_combos[current_simulation]['odom_timestamps'],
        'scan_folder': sim_combos[current_simulation]['scan_values'],
        'scan_timestamps': sim_combos[current_simulation]['scan_timestamps'],
        'initial_cartesian': sim_combos[current_simulation]['init_cartesian'],
        'final_cartesian': sim_combos[current_simulation]['fin_cartesian'],
        'type': sim_combos[current_simulation]['type'],
        'time': sim_combos[current_simulation]['time'],
        'integration': sim_combos[current_simulation]['integration'],
        'scan_file':
        sim_combos[current_simulation]['scan_values'] + '.csv',
        'occlusion_folder':
        sim_combos[current_simulation]['scan_values'],
        'single_file': '/0025.csv'
    }

    # perform some sanity checks
    pc.check_occlusion(current_data)
    # pc.check_differentiation(current_data)

    # closest points
    pp.plot_closest(current_data, 'cartesian')
    # pp.plot_closest(current_data, 'polar', 'polar')

    # complete scan at each time step
    pp.plot_complete(current_data, 'cartesian')
    # pp.plot_complete(current_data, 'polar')

    # make some nice animations
    pp.make_gifs(current_data, 'cartesian')


def process_multi_sims(simulation_combos, simulation_type):
    """process multiple simulations for a series of expts/tests"""
    for simulation in simulation_combos:
        print('Processing ' + simulation + ' ...')
        process_single_sim(simulation, simulation_type)


if __name__ == "__main__":
    SIM_TYPE = sys.argv[1]
    SIM_TYPE = SIM_TYPE.upper()
    SIM_NAMES = SIM_TYPE + '_COMBOS'
    SIM_COMBOS = getattr(sys.modules[__name__], SIM_NAMES)

    process_single_sim('v_pole_zero_tile6', SIM_TYPE)
    # process_multi_sims(SIM_COMBOS, SIM_TYPE)
