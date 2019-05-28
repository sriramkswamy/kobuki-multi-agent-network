Types of Tests and Experiments to be conducted
==============================================

Tests
-----

- O = Observer
- T = Target
- P = Pole
- W = Wall

_Each test here must include calibration_ 

1. Laser scan error test for (O, W) and (O, P)
2. Laser scan offset test for (O, P) and (O, T)
3. Polar range bias test for (P, T) and (P, O)
4. Polar angle bias test for (P, T) and (P, O)
5. Odometer origin reset test for O and T
6. Odometer orientation reset test for O and T
7. Turtlebot enters FOV of observer at speed with O and T
   1. Straight line from left FOV edge
   2. Straight line from right FOV edge
   3. Arc from left FOV edge
   4. Arc from right FOV edge
8. Turtlebot holds and starts from within the FOV with O and T
   1. Straight line from left FOV edge
   2. Straight line from right FOV edge
   3. Arc from left FOV edge
   4. Arc from right FOV edge
9. Turtlebot target T multiple circular orbits within FOV
10. Turtlebot rotating at a given position within FOV

_Note_: Repeat this experiment if the observer is changed. Need not repeat for every target change

Experiments
-----------

- V: Violet Turtlebot
- I: Indigo base station
- B: Blue Turtlebot
- G: Green Turtlebot
- Y: Yellow Turtlebot
- O: Orange Turtlebot
- R: Red Turtlebot

_Each experiment here must include calibration_. Violet is assumed to be observer in each experiment. 

Repeat each of this experiment for 2, 3, 4, and 5 targets

1. All targets orbit around a common center
2. All targets move straight in the same direction with a slight offset in the starting position
3. Targets move in a straight line but cross each others' paths without collision
4. All targets trace an arc in the same direction (motion and rotation) with a slight offset in starting position
5. All targets trace an arc but cross paths without collision
6. Targets either move straight or in an arc but in the same direction with slight offset and do not cross paths
7. Targets either move straight or in an arc but cross paths
8. Targets move arbitrarily and cross paths

The following variables can be changed to get different results:
1. For orbiting motion, the radius can be varied between trials
2. For objects moving with an offset, the direction (either motion, or rotation, or both) can be changed between trials. _Note that the directions of ALL targets have to be changed here._
3. For any target tracing an arc, the direction of rotation (clockwise or counter-clockwise) can be changed between trials
4. For objects crossing paths, sideward motion can be included to add complexity
5. Finally, objects remaining stationary, moving in a random path, or rotating at a fixed point can be included to add confusion
