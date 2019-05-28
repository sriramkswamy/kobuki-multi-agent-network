#!/usr/bin/env python

import sys
import csv
import matplotlib.pyplot as plt
from math import cos, sin


def plot_planar_polar_positions(filename):
    """Process data as x vs y"""
    x = []
    y = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            x.append(float(row[0]) * cos(float(row[1])))
            y.append(float(row[0]) * sin(float(row[1])))

    # plot the poitns
    plt.plot(x, y, 'r*', markersize=2.0)
    plt.legend(['Sensor values'])

    plt.xlabel('x position (m)')
    plt.ylabel('y position (m)')
    plt.show()


def plot_polar_sub_plots(filename):
    """Process index vs range and angle"""
    r = []
    theta = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            r.append(float(row[0]))
            theta.append(float(row[1]))

    indices = xrange(0, len(theta))

    # plot the points
    plt.figure(1)
    plt.plot(indices, r, 'r*', markersize=2.0)
    plt.legend(['Sensor values'])
    plt.xlabel('Index')
    plt.ylabel('range (m)')
    plt.savefig('range_plot.png')
    plt.close(1)

    # plot the points
    plt.figure(2)
    plt.plot(indices, theta, 'g*', markersize=2.0)
    plt.legend(['Sensor values'])
    plt.xlabel('Index')
    plt.ylabel('Angle (rad)')
    plt.savefig('angle_plot.png')
    plt.close(2)


def plot_planar_cartesian_positions(filename):
    """Process data as x vs y"""
    x = []
    y = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            x.append(float(row[0]))
            y.append(float(row[1]))

    # plot the poitns
    plt.plot(x, y, 'r*', markersize=2.0)
    plt.legend(['Sensor values'])

    plt.xlabel('x position (m)')
    plt.ylabel('y position (m)')
    plt.show()


def plot_euler_angles(filename):
    """Process data as phi, theta. psi"""
    phi = []
    theta = []
    psi = []
    with open(filename) as data_file:
        odom_data = csv.reader(data_file)
        for row in odom_data:
            phi.append(float(row[0]))
            theta.append(float(row[1]))
            psi.append(float(row[2]))

    # plot the points
    indices = xrange(0, len(psi))

    plt.plot(indices, phi, 'r*',
             indices, theta, 'b+',
             indices, psi, 'go',
             markersize=2.0)
    plt.legend(['Phi', 'Theta', 'Psi'])

    plt.xlabel('index')
    plt.ylabel('angles (radians)')
    plt.show()


if __name__ == "__main__":
    FILE_NAME = sys.argv[1]
    # plot_planar_polar_positions(FILE_NAME)
    # plot_euler_angles(FILE_NAME)
    plot_polar_sub_plots(FILE_NAME)
    # plot_planar_cartesian_positions(FILE_NAME)
