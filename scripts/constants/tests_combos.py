from constants import *
import transform_variables as tv

TESTS_COMBOS = {

    'v_pole_zero_tile6': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.05,
        'type': 'circle',
        'odom_positions': BOT_NAMES['v'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['v'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['v'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['v'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.0, 0.0]],
        'init_polar': [[0.0, 0.0]],
        'fin_cartesian': [[0.0, 0.0]],
        'fin_polar': [[0.0, 0.0]]
    },

    'v_pole_piby4_tile6': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.05,
        'type': 'circle',
        'odom_positions': BOT_NAMES['v'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['v'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['v'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['v'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.0, 0.0]],
        'init_polar': [[0.0, 0.0]],
        'fin_cartesian': [[0.0, 0.0]],
        'fin_polar': [[0.0, 0.0]]
    },

    'v_pole_3piby8_tile6': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.05,
        'type': 'circle',
        'odom_positions': BOT_NAMES['v'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['v'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['v'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['v'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.0, 0.0]],
        'init_polar': [[0.0, 0.0]],
        'fin_cartesian': [[0.0, 0.0]],
        'fin_polar': [[0.0, 0.0]]
    },

    'v_pole_5piby8_tile6': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.05,
        'type': 'circle',
        'odom_positions': BOT_NAMES['v'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['v'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['v'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['v'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.0, 0.0]],
        'init_polar': [[0.0, 0.0]],
        'fin_cartesian': [[0.0, 0.0]],
        'fin_polar': [[0.0, 0.0]]
    },

    'v_pole_3piby4_tile6': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.05,
        'type': 'circle',
        'odom_positions': BOT_NAMES['v'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['v'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['v'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['v'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.0, 0.0]],
        'init_polar': [[0.0, 0.0]],
        'fin_cartesian': [[0.0, 0.0]],
        'fin_polar': [[0.0, 0.0]]
    },

    'vb_polar_pose01': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.5631, 0.2332]],
        'init_polar': [[0.6096, 0.3927]],
        'fin_cartesian': [[0.5631, 0.2332]],
        'fin_polar': [[0.6096, 0.3927]]
    },

    'vb_polar_pose02': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.431, 0.431]],
        'init_polar': [[0.6096, 0.7854]],
        'fin_cartesian': [[0.431, 0.431]],
        'fin_polar': [[0.6096, 0.7854]]
    },

    'vb_polar_pose03': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.646, 0.646]],
        'init_polar': [[0.9144, 0.7854]],
        'fin_cartesian': [[0.646, 0.646]],
        'fin_polar': [[0.9144, 0.7854]]
    },

    'vb_polar_pose04': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.2332, 0.5632]],
        'init_polar': [[0.6096, 1.1781]],
        'fin_cartesian': [[0.2332, 0.5632]],
        'fin_polar': [[0.6096, 1.1781]]
    },

    'vr_pole_straight_fast_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['r'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['r'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['r'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['r'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[0.0, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vr_pole_straight_hold_fast_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['r'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['r'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['r'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['r'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[1.2192, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vb_pole_straight_fast_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[0.6096, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vb_pole_straight_hold_fast_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[1.2192, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vb_pole_straight_slow_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[-1.7678, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vb_pole_straight_hold_slow_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[-0.5486, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vb_pole_straight_medium_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[-0.7925, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vb_pole_straight_hold_medium_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[0.3048, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vr_pole_arc_acw_medium_trial1': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['r'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['r'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['r'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['r'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-1.8288, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[0.3048, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vb_pole_straight_fast_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[0.6096, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vb_pole_straight_hold_fast_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[1.3716, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vb_pole_straight_slow_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[-1.7678, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vb_pole_straight_hold_slow_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[-0.5486, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vb_pole_straight_medium_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-2.1336, 1.8288]],
        'init_polar': [[2.4384, 2.3562]],
        'fin_cartesian': [[-0.7925, 1.8288]],
        'fin_polar': [[1.8288, 1.5708]]
    },

    'vb_pole_straight_hold_medium_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['b'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['b'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['b'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['b'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-0.9144, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[0.3048, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

    'vr_pole_arc_acw_medium_trial2': {
        'integration': False,
        'polar': True,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10.00,
        'type': 'stationary',
        'odom_positions': BOT_NAMES['r'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['r'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['r'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['v'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['r'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'scan_timestamps': BOT_NAMES['v'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[-1.8288, 1.8288]],
        'init_polar': [[2.0726, 2.0]],
        'fin_cartesian': [[0.3048, 1.8288]],
        'fin_polar': [[2.286, 0.9817]]
    },

}
