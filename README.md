# Warmup Project

## Drive in a Square
### High-level overview
The problem at hand is to make Turtlebot3 drive in a square. In my implementation, the Turtlebot3 moves directly forward for a specified period of time before stopping to rotate 90 degrees counterclockwise. This repeats until the program is shutdown, which causes Turtlebot3 to drive in a square path. 

### Functions
`__init__`: This is the basic initialization function. It establishes the node `cmd_vel_pub` and the publisher to `/cmd_vel`, as well as initializes `velocities` as a Twist message instance (with all linear and angular velocities set to 0).

`move_forward`: Set the linear x-velocity to 0.2 and publish to the topic.

`rotate`: At a high level, this function rotates the bot 90 degrees counterclockwise. First, the variables `current_angle`, `ninety_degrees`, and `previous_time` are established. `current_angle` keeps track of the angle of rotation as the bot rotates. `ninety_degrees` is a constant that represents 90 degrees in radians. `initial_time` records the time at which rotation begins. The linear x-velocity and angular z-velocity are set to 0 and 0.5, respectively, so that the robot ceases moving forward and can only rotate around the z-axis. A sleep rate of 10Hz is also declared, which controls the rate at which `current_angle` is updated. While `current_angle` is less than 90 degrees, the new velocities are published. `current_angle` is updated by calculating the angular z-velocity multiplied by the elapsed time (`current_time - initial_time`). Once the robot has rotated 90 degrees, the `while` loop exits, and the bot's angular z-velocity is set to 0 and published so that the bot stops rotating.

`run`: This function actually does the driving in a square (noise affects the accuracy of the path a bit). The bot moves forward through function `move_forward()` for 5 seconds, due to `rospy.sleep(5)`. After 5 seconds of movement, the bot rotates through function `rotate()`. This behavior repeats infinitely until the program is shut down, causing the bot to drive in a square infinitely.

### Demonstration
![GIF of Turtlebot3 driving in a square](gifs/drive_square.gif)

## Challenges

## Future Work

## Takeaways