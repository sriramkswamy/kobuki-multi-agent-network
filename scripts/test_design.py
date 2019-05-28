"""
This module contains constants and values to design the tests
"""

from math import cos, sin, acos, asin, atan, pi
import matplotlib.pyplot as plt

import rosagent.transform_variables as tv

# distances in meters
SEPARATION_X = 0.3048
SEPARATION_Y = 0.3048
SEPARATION_R = 0.3048

# inputs in terms of separations that when combined create coordinates
X_TILES = [0, -3, 3]
Y_TILES = [0, 2, 4, 6, 8, 10]
R_TILES = [2, 3, 4, 5, 6]
THETA_VALUES = [
    0, pi / 8, pi / 4, (3*pi) / 8, pi / 2, (5 * pi) / 8, (3 * pi) / 4,
    (7 * pi) / 8, pi
]
BOT_DIAMETER = 0.254


def make_combos(first_list, second_list):
    """provides a list which is a combination of one item from given lists"""
    combo_list = []
    for first_item in first_list:
        for second_item in second_list:
            combo_list.append([first_item, second_item])
    return combo_list


def plot_cartesian(plot_x, plot_y, plot_name, fig_num=1, fig_show=False):
    """creates a plot of the provided values in cartesian coordinates"""
    plt.figure(fig_num)
    legends = []

    # set the axes limit
    axes = plt.gca()
    axes.set_xlim([-4, 4])
    axes.set_ylim([-2, 4])

    # plot the points and the observer
    plt.plot(plot_x, plot_y, 'o',
             markersize=2.5)
    legends.append('Positions')

    # plot the observer
    fov_min = pi / 6
    fov_max = (5*pi) / 6
    fov_length = 2
    observer = plt.Circle((0, 0), BOT_DIAMETER / 2, color='b')
    axes.add_artist(observer)
    plt.plot(0, 0, 'bo')
    legends.append('Observer')

    # plot the field of view
    line_min_x = [0, fov_length * cos(fov_min)]
    line_min_y = [0, fov_length * sin(fov_min)]
    line_max_x = [0, fov_length * cos(fov_max)]
    line_max_y = [0, fov_length * sin(fov_max)]
    plt.plot(line_min_x, line_min_y, 'g.-')
    legends.append('FOV min')
    plt.plot(line_max_x, line_max_y, 'r.-')
    legends.append('FOV max')

    # update labels and legends
    plt.xlabel('x position (m)')
    plt.ylabel('y position (m)')
    plt.legend(legends,
               bbox_to_anchor=(1, 1),
               loc='lower right', ncol=2, borderaxespad=1.5)
    if fig_show:
        plt.show(fig_num)
    plt.savefig(plot_name + '.png', bbox_inches='tight')
    plt.savefig(plot_name + '.eps', bbox_inches='tight')
    plt.close(fig_num)


def create_test_map(test_x, test_y, test_r, test_theta):
    """creates a map of the setup used for testing"""
    polar_combo = make_combos(test_r, test_theta)
    cartesian_combo = make_combos(test_x, test_y)

    r_values = []
    theta_values = []
    polar_x_values = []
    polar_y_values = []
    polar_x_coordinates = []
    polar_y_coordinates = []
    cartesian_x_values = []
    cartesian_y_values = []
    cartesian_x_coordinates = []
    cartesian_y_coordinates = []

    for polar_coordinate in polar_combo:
        r_values.append(polar_coordinate[0] * SEPARATION_R)
        theta_values.append(polar_coordinate[1])
        temp_x, temp_y = tv.polar_to_cartesian(polar_coordinate[0],
                                               polar_coordinate[1])
        polar_x_values.append(temp_x)
        polar_y_values.append(temp_y)

    for cartesian_coordinate in cartesian_combo:
        cartesian_x_values.append(cartesian_coordinate[0])
        cartesian_y_values.append(cartesian_coordinate[1])

    polar_x_coordinates = [x * SEPARATION_X for x in polar_x_values]
    polar_y_coordinates = [y * SEPARATION_Y for y in polar_y_values]
    cartesian_x_coordinates = [x * SEPARATION_X for x in cartesian_x_values]
    cartesian_y_coordinates = [y * SEPARATION_Y for y in cartesian_y_values]

    plot_cartesian(
        polar_x_coordinates, polar_y_coordinates, 'polar_test', fig_num=1)
    plot_cartesian(
        cartesian_x_coordinates,
        cartesian_y_coordinates,
        'cartesian_test',
        fig_num=2)

    return (polar_combo, cartesian_combo,
            r_values, theta_values,
            polar_x_coordinates, polar_y_coordinates,
            cartesian_x_coordinates, cartesian_y_coordinates)


if __name__ == '__main__':
    (polar_combination, cartesian_combination,
     r_values, theta_values,
     polar_x_metres, polar_y_metres,
     cartesian_x_metres, cartesian_y_metres) = create_test_map(
         X_TILES, Y_TILES, R_TILES, THETA_VALUES)

    polar_x_inches = tv.metres_to_inches(polar_x_metres, measure_is_list=True)
    polar_y_inches = tv.metres_to_inches(polar_y_metres, measure_is_list=True)
    cartesian_x_inches = tv.metres_to_inches(cartesian_x_metres,
                                             measure_is_list=True)
    cartesian_y_inches = tv.metres_to_inches(cartesian_y_metres,
                                             measure_is_list=True)
