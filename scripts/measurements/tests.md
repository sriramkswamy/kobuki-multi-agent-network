Tests data
==========

Report data in either polar or cartesian coordinates, whichever is convenient but make sure to report the units.

- O: Observer
- T: Target
- P: Pole
- W: Wall

- cal.: calibrate
- off.: offset
- st.: start
- pos.: position

- _ft_: feet
- _in_: inches
- _m_: metres
- _cm_: centimeters
- _rad_: radians
- _deg_: degrees

Laser scan error test
---------------------

Data to check from results:
- Variance in wall measurement should not be more than a few centimeters
- Estimate of pole position after adjusting the offset matches manual measurement

| Name    | O pos.         | P pos.         | Wall pos.      | O P off.          |
| ---     | --             | --             | --             | --                |
| trial01 | [12 in, 12 in] | [24 in, 24 in] | [48 in, 48 in] | [0.02 in, 0.2 in] |

Laser scan offset test
----------------------

Data to check from results:
- Estimate of pole position after adjusting the offset matches manual measurement
- Estimate of target position after adjusting the offset and clustering the point cloud matches manual measurement

| Name    | O pos.         | P pos.         | T pos.         | O P offset        | O T offset        |
| ---     | --             | --             | --             | --                | --                |
| trial01 | [12 in, 12 in] | [24 in, 24 in] | [48 in, 48 in] | [0.02 in, 0.2 in] | [0.02 in, 0.2 in] |

Polar range and angle bias tests
--------------------------------

Data to check from results:
- Estimate of pole position after adjusting offset matches manual measurement
- Bias/offset remains consistent for all ranges for a specified angle. If not, plot the distribution
- Bias/offset remains consistent for all angles for a specified range. If not, plot the distribution

| Name    | O pos.          | P pos.          | O P off.           |
| ---     | --              | --              | --                 |
| trial01 | [12 in, 12 rad] | [24 in, 24 rad] | [0.02 in, 0.2 rad] |

Odometer position and orientation reset tests
---------------------------------------------

Data to check from results:
- Odometer reports zero position after resetting irrespective of changing location between resets
- Odometer reports zero orientation after resetting irrespective of changing orientation between resets

| Name    | O pos.          | Odometer pos.   | O offset           |
| ---     | --              | --              | --                 |
| trial01 | [12 in, 12 rad] | [24 in, 24 rad] | [0.02 in, 0.2 rad] |

Turtlebot tests
---------------

Data to check from results:
- Odometer trajectory after adjusting for offset matches trajectory produced by observer laser scan
- There is no occlusion of data points

| Name    | O cal. | O P cal. | O P off. | T cal. | T P cal. | T P off. | T st. | T P st. | T P st. off. | T end | T P end | T P end off. |
|-------  |------  |--------  |--------  |------  |--------  |--------  |-----  |-------  |------------  |-----  |-------  |------------  |
| trial01 |        |          |          |        |          |          |       |         |              |       |         |              |
