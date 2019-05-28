# Various simulations for the turtlebots

Known variables

| Variable | Value |
|--|--|
|`pose01` | [-0.7366, 1.4478] |
|`pose02` | [-0.7366, 1.8161] |
|`pose03` | [-0.7366, 2.225] |
|`pose04` | [-0.7366, 2.489] |
|`pose05` | [-0.7366, 2.8956] |
|`pose06` | [0.127, 2.225] |
|`pose07` | [1.003, 2.8956] |
|`pose08` | [0.127, 0.7112] |
|`pose09` | [-0.7366, 0.7112] |
|`pose10` | [0.127, 3.711] |

Agent nomenclature:

|Indicator | Name |
|--|--|
| 'v' | violet |
| 'b' | blue |
| 'g' | green |
| 'y' | yellow |
| 'o' | orange |
| 'r' | red |

## Tests

### Stationary tests

- One bot is located at each of the 10 positions and its odometry positions are noted
- They should all remain (0,0)
- The orientation depends on where the bot is facing *when the ros core was started*
- Reset the bot after every change in position

## Orientation tests

- The default orientation should be 0 radians in the direction the kinect camera is pointed
- Reset the bot after every change in orientation

## Rotation tests

- This isolates one degree of freedom
- Let the bot rotate in both clockwise and anti-clockwise directions
- The angles reported should always remain within -pi and pi radians
- The 0 radian location depends on the bot's orientation when started

## Straight line forward motion

- This isolates another degree of freedom
- Start the bot at any position and orientation
- Provide a control input for only linear x velocity
- It should follow a straight line path without change in orientation

## Straight line sideward motion

- This isolates the final degree of freedom
- Start the bot at any position and orientation
- Provide a control input for only linear y velocity
- It should follow a straight line path without change in orientation

## Simulations

### Bots moving straight with an offset

- Place bots in different positions facing the same direction
- Provide only constant velocity linear control inputs in one direction for both the bots
- The bots should move together with an offset

### Bots moving straight crossing

- Place bots in different positions facing the different directions
- Provide only constant velocity linear control inputs in one direction for both the bots
- The bots should cross each other or collide depending upon initial orientation

### Bots moving with an arc offset

- Place bots in different positions facing the same direction
- Provide constant velocity linear control and constant angular velocity control inputs for both the bots
- The bots should form an arc with an offset

### Bots revolving or colliding

- Place bots in different positions facing different directions
- Provide constant velocity linear control and constant angular velocity control inputs for both the bots
- The bots should form an arc with an either orbiting each other or colliding
