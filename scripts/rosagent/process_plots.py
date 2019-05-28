"""
This module contains the functions for post processing for plotting
"""

import os
import sys
import inspect
import csv
import json
from math import cos, sin, acos, asin, atan, pi
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint

import rosagent.parse_files as pf
import rosagent.perform_checks as pc
import rosagent.transform_variables as tv


def calc_odom(current_sim_data, coordinates):
    """returns the odometry values to be plotted"""
    data_folder = current_sim_data['data_folder']
    init_positions = current_sim_data['initial_cartesian']

    if type(current_sim_data['odom_positions']) is list:
        num_bots = len(current_sim_data['odom_positions'])
    else:
        num_bots = 1

    odom_x = [None] * num_bots
    odom_y = [None] * num_bots
    odom_psi = [None] * num_bots

    current_bot = 0
    if type(current_sim_data['odom_positions']) is list:
        for odom_positions in current_sim_data['odom_positions']:
            if coordinates is 'polar':
                odom_x[current_bot], odom_y[current_bot] = pf.csv_2d_polar(
                    os.path.join(data_folder, odom_positions))
            else:
                odom_x[current_bot], odom_y[current_bot] = pf.csv_2d_cartesian(
                    os.path.join(data_folder, odom_positions))

            odom_x[current_bot], odom_y[current_bot] = tv.odom_offset(
                odom_x[current_bot], odom_y[current_bot],
                init_positions[current_bot])
            odom_psi[current_bot] = pf.csv_orientation(
                os.path.join(data_folder, odom_positions))

            current_bot += 1

    else:
        odom_positions = current_sim_data['odom_positions']
        if coordinates is 'polar':
            odom_x[current_bot], odom_y[current_bot] = pf.csv_2d_polar(
                os.path.join(data_folder, odom_positions))
        else:
            odom_x, odom_y = pf.csv_2d_cartesian(os.path.join(data_folder, odom_positions))

        odom_x, odom_y = tv.odom_offset(
            odom_x, odom_y, init_positions[current_bot])
        odom_psi = pf.csv_orientation(os.path.join(data_folder, odom_positions))

    return odom_x, odom_y, odom_psi


def plot_closest(current_sim_data, coordinates):
    """Plots the closest point scan values in sepcified coordinates"""
    data_folder = current_sim_data['data_folder']
    plots_folder = current_sim_data['plots_folder']
    scan_file = current_sim_data['scan_file']
    legends = []

    try:
        os.mkdir(plots_folder)
    except OSError:
        print 'Plots folder exists or it cannot be created'

    # start a new figure
    plt.figure(1)

    # set axes limit
    axes = plt.gca()
    axes.set_xlim([-5, 5])
    axes.set_ylim([-2, 5])

    if coordinates is 'polar':
        scan_x, scan_y = pf.csv_2d_polar(os.path.join(data_folder, scan_file),
                                         polar=True, scan=True)
    else:
        scan_x, scan_y = pf.csv_2d_cartesian(os.path.join(data_folder, scan_file),
                                             polar=True, scan=True)
        scan_temp = np.multiply(-1, scan_y)
        scan_y = scan_x
        scan_x = scan_temp
        plt.xlabel('x position (m)')
        plt.ylabel('y position (m)')

    if type(current_sim_data['odom_dcm']) is list:
        num_bots = len(current_sim_data['odom_dcm'])
        dcm = [None] * num_bots
        for bot in xrange(num_bots):
            dcm[bot] = pf.csv_matrix(os.path.join(data_folder, current_sim_data['odom_dcm'][bot]))
    else:
        dcm = pf.csv_matrix(os.path.join(data_folder, current_sim_data['odom_dcm']))

    odom_x, odom_y, odom_psi = calc_odom(current_sim_data, coordinates)

    # set axes and some lengths
    line_length = 0.1

    # plot all the odometry points
    if type(current_sim_data['odom_positions']) is list:
        num_bots = len(odom_x)
        for bot in xrange(num_bots):
            bot_x = odom_x[bot]
            bot_y = odom_y[bot]
            plt.plot(bot_x, bot_y, '+', markersize=1.5)
            legends.append('Odometry Robot ' + str(bot + 1))
    else:
        plt.plot(odom_x, odom_y, '+', markersize=1.5)
        legends.append('Odometry positions')

    # plot all the laser scan points
    plt.plot(scan_x, scan_y, "c*", 0, 0, 'yo', markersize=2.5)
    legends.append('Laser scanner')
    legends.append('Observer')

    # plot a line indicating the initial orientation
    if type(current_sim_data['odom_positions']) is list:
        num_bots = len(odom_x)
        for bot in xrange(num_bots):
            bot_x = odom_x[bot]
            bot_y = odom_y[bot]
            bot_psi = odom_psi[bot]

            # initial orientation
            init_x = [bot_x[0]]
            init_y = [bot_y[0]]
            init_angle = bot_psi[0]
            init_x.append(init_x[0] + line_length*cos(init_angle))
            init_y.append(init_y[0] + line_length*sin(init_angle))
            plt.plot(init_x, init_y, 'g-')
            legends.append('Initial orientation (Robot ' + str(bot + 1) + ')')

            # final orientation
            final_x = [bot_x[len(bot_x) - 1]]
            final_y = [bot_y[len(bot_y) - 1]]
            final_angle = bot_psi[len(bot_psi) - 1]
            final_x.append(final_x[0] + line_length*cos(final_angle))
            final_y.append(final_y[0] + line_length*sin(final_angle))
            plt.plot(final_x, final_y, 'r-')
            legends.append('Final orientation(Robot ' + str(bot + 1) + ')')
    else:
        # initial orientation
        init_x = [odom_x[0]]
        init_y = [odom_y[0]]
        init_angle = odom_psi[0]
        init_x.append(init_x[0] + line_length*cos(init_angle))
        init_y.append(init_y[0] + line_length*sin(init_angle))
        plt.plot(init_x, init_y, 'g-')
        legends.append('Initial orientation')

        # final orientation
        final_x = [odom_x[len(odom_x) - 1]]
        final_y = [odom_y[len(odom_x) - 1]]
        final_angle = odom_psi[len(odom_psi) - 1]
        final_x.append(final_x[0] + line_length*cos(final_angle))
        final_y.append(final_y[0] + line_length*sin(final_angle))
        plt.plot(final_x, final_y, 'r-')
        legends.append('Final orientation')

    plt.legend(legends, bbox_to_anchor=(1, 1), loc='lower right',
               ncol=2, borderaxespad=1.5)
    plt.savefig(os.path.join(plots_folder, coordinates + '_odom_scan_closest.png'),
                bbox_inches='tight')
    plt.savefig(os.path.join(plots_folder, coordinates + '_odom_scan_closest.eps'),
                bbox_inches='tight')
    # plt.show(1)
    plt.close(1)


def plot_complete(current_sim_data, coordinates):
    """Plots the stem complete point cloud scan values in said coordinates"""
    data_folder = current_sim_data['data_folder']
    plots_folder = current_sim_data['plots_folder']
    scan_folder = current_sim_data['scan_folder']
    legends = []

    path, dirs, files = next(os.walk(data_folder + scan_folder))
    time_steps = len(files)

    try:
        os.mkdir(plots_folder)
    except OSError:
        print 'Plots folder exists or it cannot be created'

    try:
        os.mkdir(os.path.join(plots_folder, scan_folder))
    except OSError:
        print 'Combo folder exists or it cannot be created'

    scan_x = [None] * time_steps
    scan_y = [None] * time_steps
    step = 0
    while step < time_steps:
        index = step + 1
        format_string = '%04d' % index

        plt.figure(step + 1)
        print 'Step: ' + format_string

        # set axes limit
        axes = plt.gca()
        axes.set_xlim([-5, 5])
        axes.set_ylim([-2, 5])

        if coordinates is 'polar':
            scan_x[step], scan_y[step] = pf.csv_2d_polar(
                os.path.join(data_folder, scan_folder, format_string + '.csv'),
                polar=True, scan=True)
        else:
            scan_x[step], scan_y[step] = pf.csv_2d_cartesian(
                os.path.join(data_folder, scan_folder, format_string + '.csv'),
                polar=True, scan=True)
            scan_temp = np.multiply(-1, scan_y[step])
            scan_y[step] = scan_x[step]
            scan_x[step] = scan_temp
            plt.xlabel('x position (m)')
            plt.ylabel('y position (m)')

        if type(current_sim_data['odom_dcm']) is list:
            num_bots = len(current_sim_data['odom_dcm'])
            dcm = [None] * num_bots
            for bot in xrange(num_bots):
                dcm[bot] = pf.csv_matrix(os.path.join(data_folder, current_sim_data['odom_dcm'][bot]))
        else:
            dcm = pf.csv_matrix(os.path.join(data_folder, current_sim_data['odom_dcm']))

        odom_x, odom_y, odom_psi = calc_odom(current_sim_data, coordinates)

        # plot all the points
        line_length = 0.2
        if type(current_sim_data['odom_positions']) is list:
            num_bots = len(odom_x)
            for bot in xrange(num_bots):
                bot_x = odom_x[bot]
                bot_y = odom_y[bot]
                plt.plot(bot_x, bot_y, '+', markersize=1.5)
                legends.append('Odometry Robot ' + str(bot + 1))
        else:
            plt.plot(odom_x, odom_y, '+', markersize=1.5)
            legends.append('Odometry positions')

        plt.plot(scan_x[step], scan_y[step], "c*", 0, 0, 'yo', markersize=2.5)
        legends.append('Laser scanner')
        legends.append('Observer')

        # plot a line indicating the initial orientation
        if type(current_sim_data['odom_positions']) is list:
            num_bots = len(odom_x)
            for bot in xrange(num_bots):
                bot_x = odom_x[bot]
                bot_y = odom_y[bot]
                bot_psi = odom_psi[bot]

                # initial orientation
                init_x = [bot_x[0]]
                init_y = [bot_y[0]]
                init_angle = bot_psi[0]
                init_x.append(init_x[0] + line_length*cos(init_angle))
                init_y.append(init_y[0] + line_length*sin(init_angle))
                plt.plot(init_x, init_y, 'g-')

                # final orientation
                legends.append('Initial orientation (Robot' + str(bot + 1) + ')')
                final_x = [bot_x[len(bot_x) - 1]]
                final_y = [bot_y[len(bot_y) - 1]]
                final_angle = bot_psi[len(bot_psi) - 1]
                final_x.append(final_x[0] + line_length*cos(final_angle))
                final_y.append(final_y[0] + line_length*sin(final_angle))
                plt.plot(final_x, final_y, 'r-')
                legends.append('Final orientation (Robot' + str(bot + 1) + ')')
        else:
            # initial orientation
            init_x = [odom_x[0]]
            init_y = [odom_y[0]]
            init_angle = odom_psi[0]
            init_x.append(init_x[0] + line_length*cos(init_angle))
            init_y.append(init_y[0] + line_length*sin(init_angle))
            plt.plot(init_x, init_y, 'g-')
            legends.append('Initial orientation')

            # final orientaiton
            final_x = [odom_x[len(odom_x) - 1]]
            final_y = [odom_y[len(odom_x) - 1]]
            final_angle = odom_psi[len(odom_psi) - 1]
            final_x.append(final_x[0] + line_length*cos(final_angle))
            final_y.append(final_y[0] + line_length*sin(final_angle))
            plt.plot(final_x, final_y, 'r-')
            legends.append('Final orientation')

        plt.legend(legends, bbox_to_anchor=(1, 1), loc='lower right', ncol=2)
        plt.savefig(os.path.join(plots_folder, scan_folder,
                    coordinates + '_odom_scan_complete_' + format_string + '.png'),
                    bbox_inches='tight')
        plt.savefig(os.path.join(plots_folder, scan_folder,
                    coordinates + '_odom_scan_complete_' + format_string + '.eps'),
                    bbox_inches='tight')
        # plt.show(step + 1)
        plt.close(step + 1)

        step += 1


def make_gifs(current_sim_data, coordinates):
    """Make GIFs from scan values"""
    print('Making gifs. This might take a while ...')
    scan_folder = current_sim_data['scan_folder']
    plots_folder = current_sim_data['plots_folder']
    plot_file_wildcard = os.path.join(plots_folder, scan_folder, '') + coordinates + '*.png'
    gif_file_name = os.path.join(plots_folder, coordinates + '_timeline.gif')
    loop_delay = '10'
    loop_continuous = '0'
    shell_command = ('convert -delay ' + loop_delay +
                     ' -loop ' + loop_continuous + ' ' +
                     plot_file_wildcard + ' ' + gif_file_name)
    os.system(shell_command)
