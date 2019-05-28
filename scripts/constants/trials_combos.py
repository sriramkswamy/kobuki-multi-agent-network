from constants import *
import transform_variables as tv

TRIALS_COMBOS = {
    'RO_circleobserve': {
        'integration': False,
        'polar': False,
        'wall_gating_dist': WALL_DIST_M,
        'time': 10,
        'type': 'circle',
        'odom_positions': BOT_NAMES['r'] + ODOM_POSITIONS,
        'odom_velocities': BOT_NAMES['r'] + ODOM_VELOCITIES,
        'odom_dcm': BOT_NAMES['r'] + ODOM_DCM,
        'odom_timestamps': BOT_NAMES['r'] + ODOM_TIMESTAMPS,
        'odom_quaternions': BOT_NAMES['r'] + ODOM_QUATERNIONS,
        'scan_values': SCAN_LOCATION['o'],
        'scan_timestamps': BOT_NAMES['o'] + SCAN_TIMESTAMPS,
        'target_offset_polar': [],
        'target_offset_cartesian': [],
        'observer_offset_polar': [],
        'observer_offset_cartesian': [],
        'init_cartesian': [[0.0, 0.0]],
        'init_polar': [[0.0, 0.0]],
        'fin_cartesian': [[0.0, 0.0]],
        'fin_polar': [[0.0, 0.0]]
    }
}
