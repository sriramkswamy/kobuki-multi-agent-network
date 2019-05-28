from constants import *

OLD_EXPTS_COMBOS = {

    'vbgr_pattern': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 12.88,
        'type': 'pattern',
        'odom_positions': [
            BOT_NAMES['bgr'][0] + ODOM_POSITIONS,
            BOT_NAMES['bgr'][1] + ODOM_POSITIONS,
            BOT_NAMES['bgr'][2] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bgr'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bgr'][1] + ODOM_VELOCITIES,
            BOT_NAMES['bgr'][2] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bgr'][0] + ODOM_DCM,
            BOT_NAMES['bgr'][1] + ODOM_DCM,
            BOT_NAMES['bgr'][2] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bgr'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bgr'][1] + ODOM_QUATERNIONS,
            BOT_NAMES['bgr'][2] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[-0.7366, 1.4478], [1.003, 2.8956], [0.127, 2.225]],
        'init_polar': [[-0.7366, 1.4478], [1.003, 2.8956], [0.127, 2.225]],
        'fin_cartesian': [[-1.5113, 3.825], [0.404, 2.56], [0.127, 2.225]],
        'fin_polar': [[-1.5113, 3.825], [0.404, 2.56], [0.127, 2.225]]
    },

    'vbgyr_pattern': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 12.61,
        'type': 'pattern',
        'odom_positions': [
            BOT_NAMES['bgyr'][0] + ODOM_POSITIONS,
            BOT_NAMES['bgyr'][1] + ODOM_POSITIONS,
            BOT_NAMES['bgyr'][2] + ODOM_POSITIONS,
            BOT_NAMES['bgyr'][3] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bgyr'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bgyr'][1] + ODOM_VELOCITIES,
            BOT_NAMES['bgyr'][2] + ODOM_VELOCITIES,
            BOT_NAMES['bgyr'][3] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bgyr'][0] + ODOM_DCM,
            BOT_NAMES['bgyr'][1] + ODOM_DCM,
            BOT_NAMES['bgyr'][2] + ODOM_DCM,
            BOT_NAMES['bgyr'][3] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bgyr'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyr'][1] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyr'][2] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyr'][3] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [
            [-0.7366, 1.4478], [1.003, 2.8956],
            [-0.7366, 2.8956], [0.127, 2.225]
        ],
        'init_polar': [
            [-0.7366, 1.4478], [1.003, 2.8956],
            [-0.7366, 2.8956], [0.127, 2.225]
        ],
        'fin_cartesian': [
            [0.4953, 2.7457], [-0.3048, 4.7254],
            [-0.4191, 1.4478], [0.127, 2.225]
        ],
        'fin_polar': [
            [0.4953, 2.7457], [-0.3048, 4.7254],
            [-0.4191, 1.4478], [0.127, 2.225]
        ]
    },

    'vbgyor_pattern': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 12.88,
        'type': 'pattern',
        'odom_positions': [
            BOT_NAMES['bgyor'][0] + ODOM_POSITIONS,
            BOT_NAMES['bgyor'][1] + ODOM_POSITIONS,
            BOT_NAMES['bgyor'][2] + ODOM_POSITIONS,
            BOT_NAMES['bgyor'][3] + ODOM_POSITIONS,
            BOT_NAMES['bgyor'][4] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bgyor'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bgyor'][1] + ODOM_VELOCITIES,
            BOT_NAMES['bgyor'][2] + ODOM_VELOCITIES,
            BOT_NAMES['bgyor'][3] + ODOM_VELOCITIES,
            BOT_NAMES['bgyor'][4] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bgyor'][0] + ODOM_DCM,
            BOT_NAMES['bgyor'][1] + ODOM_DCM,
            BOT_NAMES['bgyor'][2] + ODOM_DCM,
            BOT_NAMES['bgyor'][3] + ODOM_DCM,
            BOT_NAMES['bgyor'][4] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bgyor'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyor'][1] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyor'][2] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyor'][3] + ODOM_QUATERNIONS,
            BOT_NAMES['bgyor'][4] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[1.003, 2.8956], [-0.7366, 2.8956]],
        'init_polar': [[1.003, 2.8956], [-0.7366, 2.8956]],
        'fin_cartesian': [[1.8034, 1.8161], [-1.4605, 4.3968]],
        'fin_polar': [[1.8034, 1.8161], [-1.4605, 4.3968]]
    },

    'vr_semi_circle': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.05,
        'type': 'circle',
        'odom_positions': BOT_NAMES['r'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['r'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['r'] + ODOM_DCM,
        'odom_quaternions': BOT_NAMES['r'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[-0.7366, 2.8956]],
        'init_polar': [[-0.7366, 2.8956]],
        'fin_cartesian': [[1.01, 3.01]],
        'fin_polar': [[1.01, 3.01]]
    },

    'vgr_circle_opp': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 9.36,
        'type': 'circle',
        'odom_positions': [
            BOT_NAMES['gr'][0] + ODOM_POSITIONS,
            BOT_NAMES['gr'][1] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['gr'][0] + ODOM_VELOCITIES,
            BOT_NAMES['gr'][1] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['gr'][0] + ODOM_DCM,
            BOT_NAMES['gr'][1] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['gr'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['gr'][1] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[1.003, 2.8956], [-0.7366, 2.8956]],
        'init_polar': [[1.003, 2.8956], [-0.7366, 2.8956]],
        'fin_cartesian': [[1.8034, 1.8161], [-1.4605, 4.3968]],
        'fin_polar': [[1.8034, 1.8161], [-1.4605, 4.3968]]
    },

    'vbg_straight_offset': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.55,
        'type': 'straight',
        'odom_positions': [
            BOT_NAMES['bg'][0] + ODOM_POSITIONS,
            BOT_NAMES['bg'][1] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bg'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bg'][1] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bg'][0] + ODOM_DCM,
            BOT_NAMES['bg'][1] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bg'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bg'][1] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[-0.7366, 1.4478], [0.127, 2.225]],
        'init_polar': [[-0.7366, 1.4478], [0.127, 2.225]],
        'fin_cartesian': [[1.01, 1.4478], [1.9812, 2.225]],
        'fin_polar': [[1.01, 1.4478], [1.9812, 2.225]]
    },

    'vbg_straight_perpendicular': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 13.37,
        'type': 'straight',
        'odom_positions': [
            BOT_NAMES['bg'][0] + ODOM_POSITIONS,
            BOT_NAMES['bg'][1] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bg'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bg'][1] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bg'][0] + ODOM_DCM,
            BOT_NAMES['bg'][1] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bg'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bg'][1] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[-0.7366, 1.4478], [0.127, 3.711]],
        'init_polar': [[-0.7366, 1.4478], [0.127, 3.711]],
        'fin_cartesian': [[1.01, 1.4478], [0.127, 2.082]],
        'fin_polar': [[1.01, 1.4478], [0.127, 2.082]]
    },

    'vbg_straight_anti_parallel': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 12.81,
        'type': 'straight',
        'odom_positions': [
            BOT_NAMES['bg'][0] + ODOM_POSITIONS,
            BOT_NAMES['bg'][1] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bg'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bg'][1] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bg'][0] + ODOM_DCM,
            BOT_NAMES['bg'][1] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bg'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bg'][1] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[-0.7366, 1.4478], [1.003, 2.8956]],
        'init_polar': [[-0.7366, 1.4478], [1.003, 2.8956]],
        'fin_cartesian': [[-0.7366, 3.17], [1.003, 1.2192]],
        'fin_polar': [[-0.7366, 3.17], [1.003, 1.2192]]
    },

    'vbg_straight_cross': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 12.96,
        'type': 'straight',
        'odom_positions': [
            BOT_NAMES['bg'][0] + ODOM_POSITIONS,
            BOT_NAMES['bg'][1] + ODOM_POSITIONS
        ],
        'odom_velocities': [
            BOT_NAMES['bg'][0] + ODOM_VELOCITIES,
            BOT_NAMES['bg'][1] + ODOM_VELOCITIES
        ],
        'odom_dcm': [
            BOT_NAMES['bg'][0] + ODOM_DCM,
            BOT_NAMES['bg'][1] + ODOM_DCM
        ],
        'odom_quaternions': [
            BOT_NAMES['bg'][0] + ODOM_QUATERNIONS,
            BOT_NAMES['bg'][1] + ODOM_QUATERNIONS
        ],
        'scan_values': SCAN_LOCATION['v'],
        'odom_offset_polar': [],
        'odom_offset_cartesian': [],
        'scan_offset_polar': [],
        'scan_offset_cartesian': [],
        'init_cartesian': [[-0.7366, 2.489], [1.003, 2.8956]],
        'init_polar': [[-0.7366, 2.489], [1.003, 2.8956]],
        'fin_cartesian': [[0.6985, 1.4859], [-0.2032, 1.8288]],
        'fin_polar': [[0.6985, 1.4859], [-0.2032, 1.8288]]
    },

}
