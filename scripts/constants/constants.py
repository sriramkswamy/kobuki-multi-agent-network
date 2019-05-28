import os
import sys
import inspect
from math import pi

# folder containing the output data
EXPTS_BASE_DATA_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
EXPTS_BASE_DATA_FOLDER = os.path.join(EXPTS_BASE_DATA_FOLDER, 'data', 'expts', '')

# folder containing the output plots
EXPTS_BASE_PLOTS_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
EXPTS_BASE_PLOTS_FOLDER = os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'plots', 'expts', '')

# folder containing the output data
TESTS_BASE_DATA_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
TESTS_BASE_DATA_FOLDER = os.path.join(TESTS_BASE_DATA_FOLDER, 'data', 'tests', '')

# folder containing the output plots
TESTS_BASE_PLOTS_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
TESTS_BASE_PLOTS_FOLDER = os.path.join(TESTS_BASE_PLOTS_FOLDER, 'plots', 'tests', '')

# folder containing the output data
TRIALS_BASE_DATA_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
TRIALS_BASE_DATA_FOLDER = os.path.join(TRIALS_BASE_DATA_FOLDER, 'data', 'trials', '')

# folder containing the output plots
TRIALS_BASE_PLOTS_FOLDER = (os.path.realpath(
    os.path.abspath(
        os.path.split(
            os.path.dirname(inspect.getfile(inspect.currentframe())))[0])))
TRIALS_BASE_PLOTS_FOLDER = os.path.join(TRIALS_BASE_PLOTS_FOLDER, 'plots', 'trials', '')

# dictionary to map proper folders for old experiments
OLD_EXPTS_DATA_FOLDER = {
    'vbgr_pattern': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbgr_pattern', ''),
    'vbgyr_pattern': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbgyr_pattern', ''),
    'vbgyor_pattern': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbgyor_pattern', ''),
    'vr_semi_circle': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vr_semi_circle', ''),
    'vgr_circle_opp': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vgr_circle_opp', ''),
    'vbg_straight_offset': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbg_straight_offset', ''),
    'vbg_straight_cross': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbg_straight_cross', ''),
    'vbg_straight_anti_parallel': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbg_straight_anti_parallel', ''),
    'vbg_straight_perpendicular': os.path.join(EXPTS_BASE_DATA_FOLDER, 'vbg_straight_perpendicular', '')
}
OLD_EXPTS_PLOTS_FOLDER = {
    'vbgr_pattern': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbgr_pattern', ''),
    'vbgyr_pattern': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbgyr_pattern', ''),
    'vbgyor_pattern': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbgyor_pattern', ''),
    'vr_semi_circle': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vr_semi_circle', ''),
    'vgr_circle_opp': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vgr_circle_opp', ''),
    'vbg_straight_offset': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbg_straight_offset', ''),
    'vbg_straight_cross': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbg_straight_cross', ''),
    'vbg_straight_anti_parallel': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbg_straight_anti_parallel', ''),
    'vbg_straight_perpendicular': os.path.join(EXPTS_BASE_PLOTS_FOLDER, 'vbg_straight_perpendicular', '')
}

# dictionary to map proper folders for tests
TESTS_DATA_FOLDER = {
    'vb_polar_pose01': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_polar_pose01', ''),
    'vb_polar_pose02': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_polar_pose02', ''),
    'vb_polar_pose03': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_polar_pose03', ''),
    'vb_polar_pose04': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_polar_pose04', ''),
    'v_pole_zero_tile6': os.path.join(TESTS_BASE_DATA_FOLDER, 'v_pole_zero_tile6', ''),
    'v_pole_piby4_tile6': os.path.join(TESTS_BASE_DATA_FOLDER, 'v_pole_piby4_tile6', ''),
    'v_pole_3piby8_tile6': os.path.join(TESTS_BASE_DATA_FOLDER, 'v_pole_3piby8_tile6', ''),
    'v_pole_5piby8_tile6': os.path.join(TESTS_BASE_DATA_FOLDER, 'v_pole_5piby8_tile6', ''),
    'v_pole_3piby4_tile6': os.path.join(TESTS_BASE_DATA_FOLDER, 'v_pole_3piby4_tile6', ''),
    'vr_pole_straight_fast_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vr_pole_straight_fast_trial1', ''),
    'vr_pole_straight_hold_fast_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vr_pole_straight_hold_fast_trial1', ''),
    'vb_pole_straight_fast_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_fast_trial1', ''),
    'vb_pole_straight_hold_fast_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_hold_fast_trial1', ''),
    'vb_pole_straight_slow_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_slow_trial1', ''),
    'vb_pole_straight_hold_slow_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_hold_slow_trial1', ''),
    'vb_pole_straight_medium_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_medium_trial1', ''),
    'vb_pole_straight_hold_medium_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_hold_medium_trial1', ''),
    'vb_pole_straight_fast_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_fast_trial2', ''),
    'vb_pole_straight_hold_fast_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_hold_fast_trial2', ''),
    'vb_pole_straight_slow_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_slow_trial2', ''),
    'vb_pole_straight_hold_slow_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_hold_slow_trial2', ''),
    'vb_pole_straight_medium_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_medium_trial2', ''),
    'vb_pole_straight_hold_medium_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vb_pole_straight_hold_medium_trial2', ''),
    'vr_pole_arc_acw_medium_trial1': os.path.join(TESTS_BASE_DATA_FOLDER, 'vr_pole_arc_acw_medium_trial1', ''),
    'vr_pole_arc_acw_medium_trial2': os.path.join(TESTS_BASE_DATA_FOLDER, 'vr_pole_arc_acw_medium_trial2', '')
}
TESTS_PLOTS_FOLDER = {
    'vb_polar_pose01': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_polar_pose01', ''),
    'vb_polar_pose02': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_polar_pose02', ''),
    'vb_polar_pose03': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_polar_pose03', ''),
    'vb_polar_pose04': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_polar_pose04', ''),
    'v_pole_zero_tile6': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'v_pole_zero_tile6', ''),
    'v_pole_piby4_tile6': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'v_pole_piby4_tile6', ''),
    'v_pole_3piby8_tile6': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'v_pole_3piby8_tile6', ''),
    'v_pole_5piby8_tile6': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'v_pole_5piby8_tile6', ''),
    'v_pole_3piby4_tile6': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'v_pole_3piby4_tile6', ''),
    'vr_pole_straight_fast_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vr_pole_straight_fast_trial1', ''),
    'vr_pole_straight_hold_fast_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vr_pole_straight_hold_fast_trial1', ''),
    'vb_pole_straight_fast_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_fast_trial1', ''),
    'vb_pole_straight_hold_fast_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_hold_fast_trial1', ''),
    'vb_pole_straight_slow_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_slow_trial1', ''),
    'vb_pole_straight_hold_slow_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_hold_slow_trial1', ''),
    'vb_pole_straight_medium_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_medium_trial1', ''),
    'vb_pole_straight_hold_medium_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_hold_medium_trial1', ''),
    'vb_pole_straight_fast_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_fast_trial2', ''),
    'vb_pole_straight_hold_fast_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_hold_fast_trial2', ''),
    'vb_pole_straight_slow_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_slow_trial2', ''),
    'vb_pole_straight_hold_slow_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_hold_slow_trial2', ''),
    'vb_pole_straight_medium_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_medium_trial2', ''),
    'vb_pole_straight_hold_medium_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vb_pole_straight_hold_medium_trial2', ''),
    'vr_pole_arc_acw_medium_trial1': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vr_pole_arc_acw_medium_trial1', ''),
    'vr_pole_arc_acw_medium_trial2': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vr_pole_arc_acw_medium_trial2', '')
}

# dictionary to map proper folders for trials
TRIALS_DATA_FOLDER = {
    'RO_circleobserve': os.path.join(TRIALS_BASE_DATA_FOLDER, 'RO_circleobserve', '')
}
TRIALS_PLOTS_FOLDER = {
    'RO_circleobserve': os.path.join(TRIALS_BASE_PLOTS_FOLDER, 'RO_circleobserve', '')
}

# dictionary to map proper folders for old tests
OLD_TESTS_DATA_FOLDER = {
    'vg_pose01': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose01', ''),
    'vg_pose02': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose02', ''),
    'vg_pose03': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose03', ''),
    'vg_pose04': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose04', ''),
    'vg_pose05': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose05', ''),
    'vg_pose06': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose06', ''),
    'vg_pose07': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose07', ''),
    'vg_pose08': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose08', ''),
    'vg_pose09': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose09', ''),
    'vg_pose10': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_pose10', ''),
    'vg_orient_north': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_orient_north', ''),
    'vg_orient_south': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_orient_south', ''),
    'vg_orient_west': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_orient_west', ''),
    'vg_orient_east': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_orient_east', ''),
    'vg_straight_north': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_straight_north', ''),
    'vg_straight_south': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_straight_south', ''),
    'vg_straight_west': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_straight_west', ''),
    'vg_straight_east': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_straight_east', ''),
    'vg_full_rotate_acw': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_full_rotate_acw', ''),
    'vg_full_rotate_cw': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_full_rotate_cw', ''),
    'vg_rotate_acw': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_rotate_acw', ''),
    'vg_rotate_cw': os.path.join(TESTS_BASE_DATA_FOLDER, 'vg_rotate_cw', ''),
    'vbg_rotate_acw_same': os.path.join(TESTS_BASE_DATA_FOLDER, 'vbg_rotate_acw_same', ''),
    'vbg_rotate_cw_opp': os.path.join(TESTS_BASE_DATA_FOLDER, 'vbg_rotate_cw_opp', ''),
    'vbg_pose06_pose01_same': os.path.join(TESTS_BASE_DATA_FOLDER, 'vbg_pose06_pose01_same', ''),
    'vbg_pose06_pose01_opp': os.path.join(TESTS_BASE_DATA_FOLDER, 'vbg_pose06_pose01_opp', '')
}
OLD_TESTS_PLOTS_FOLDER = {
    'vg_pose01': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose01', ''),
    'vg_pose02': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose02', ''),
    'vg_pose03': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose03', ''),
    'vg_pose04': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose04', ''),
    'vg_pose05': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose05', ''),
    'vg_pose06': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose06', ''),
    'vg_pose07': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose07', ''),
    'vg_pose08': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose08', ''),
    'vg_pose09': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose09', ''),
    'vg_pose10': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_pose10', ''),
    'vg_orient_north': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_orient_north', ''),
    'vg_orient_south': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_orient_south', ''),
    'vg_orient_west': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_orient_west', ''),
    'vg_orient_east': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_orient_east', ''),
    'vg_straight_north': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_straight_north', ''),
    'vg_straight_south': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_straight_south', ''),
    'vg_straight_west': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_straight_west', ''),
    'vg_straight_east': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_straight_east', ''),
    'vg_full_rotate_acw': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_full_rotate_acw', ''),
    'vg_full_rotate_cw': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_full_rotate_cw', ''),
    'vg_rotate_acw': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_rotate_acw', ''),
    'vg_rotate_cw': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vg_rotate_cw', ''),
    'vbg_rotate_acw_same': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vbg_rotate_acw_same', ''),
    'vbg_rotate_cw_opp': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vbg_rotate_cw_opp', ''),
    'vbg_pose06_pose01_same': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vbg_pose06_pose01_same', ''),
    'vbg_pose06_pose01_opp': os.path.join(TESTS_BASE_PLOTS_FOLDER, 'vbg_pose06_pose01_opp', '')
}

# scan and odometry files
ODOM_POSITIONS = '_odom_positions.csv'
ODOM_VELOCITIES = '_odom_velocities.csv'
ODOM_DCM = '_odom_dcm.csv'
ODOM_QUATERNIONS = '_odom_quaternions.csv'
ODOM_TIMESTAMPS = '_odom_timestamps.csv'
SCAN_TIMESTAMPS = '_scan_timestamps.csv'

# robot names
BOT_NAMES = {
    'v': 'violet',
    'b': 'blue',
    'g': 'green',
    'y': 'yellow',
    'o': 'orange',
    'r': 'red',
    'bg': ['blue', 'green'],
    'gr': ['green', 'red'],
    'bgr': ['blue', 'green', 'red'],
    'bgyr': ['blue', 'green', 'yellow', 'red'],
    'bgyor': ['blue', 'green', 'yellow', 'orange', 'red']
}
SCAN_LOCATION = {
    'v': 'violet_scan_values',
    'b': 'blue_scan_values',
    'g': 'green_scan_values',
    'y': 'yellow_scan_values',
    'o': 'orange_scan_values',
    'r': 'red_scan_values'
}

# other physical gates/values
TILE_DIST_IN = 12
TILE_DIST_M = 0.3048
WALL_DIST_IN = 12*TILE_DIST_IN
WALL_DIST_M = 12*TILE_DIST_M
