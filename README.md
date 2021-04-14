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

## Person Follower
### High-level overview
The goal is to have Turtlebot3 follow around a person (object) as it moves around the map, provided that the object exists and is within range of the robot's LiDAR.

### Functions
`__init__`: This is the basic initialization function. It establishes the node `person_follower`, the publisher to `/cmd_vel`, the subscriber to the `/scan` topic, and initializes `velocities` as a Twist message instance (with all linear and angular velocities set to 0).

`process_scan`: This is the callback function for the subscriber defined in `__init__`. Its non-`self` parameter, `data`, records the data returned by `/scan`. The function calculates the minimum distance from the robot to the object, and takes note of the angle at which this minimum distance exists. If no object exists or the object is out of the LiDAR's range, the function simply returns. The robot rotates until it faces the object (if not already facing it), where "facing" an object is defined by the LiDAR detecting an object within the angle range 0-10 degrees. It proceeds to move forward toward the object until it is within 0.05 of a specified distance away from the object (the 0.05 margin of error helps with the noise).

`run`: This function runs the program until Rospy receives a shutdown signal.

### Demonstration
![GIF of Turtlebot3 following a cylinder object](gifs/person_follower.gif)

## Wall Follower
### High-level overview
The wall follower program makes Turtlebot3 drive along a wall from a fixed distance until it reaches another wall, where it will turn and then proceed to follow along that wall. The robot can also find a "first wall" to follow when dropped into the world.

### Functions
`__init__`: This is the basic initialization function. It establishes the node `cmd_vel_pub` and the publisher to `/cmd_vel`, as well as initializes `velocities` as a Twist message instance (with all linear and angular velocities set to 0).

`process_scan`: This is the callback function for the subscriber. It records the distance from the nearest wall for the front, front-right, and right side of the robot (as specified in the code comments). Based on these distances, the robot will first move forward towards a wall if it's not already within the specified distance from the wall. The robot will then rotate until its right side is also within the specified distance, if not already so. The robot would then move forward again, and this cycle would repeat. 

`run`: This function runs the program until a shutdown signal is received.

### Demonstration
![GIF of Turtlebot3 sort of following the walls inside a square room](gifs/wall_follower.gif)

## Challenges
I really struggled at first with figuring out how each value returned from `/scan` worked. It just wasn't clicking for some reason, so at first, I could barely even get the robot to move towards an object at all, let alone follow one. I knew what the `ranges[]` array represented, but that was pretty much it. At one point, I was trying to use attributes like `time_increment` or `range_min` in my code without realizing that they weren't significant to my task at hand. After that roadblock, I had a difficult time getting the robot to follow the object in a relatively straight path without angular velocity. Because the angular and linear velocities are handled in the same callback function that is called repeatedly, I had to figure out how to get the robot to use angular velocity only at certain positions, and same for the linear velocity. 
My other great challenge was trying to get the robot to drive at a fixed distance from the wall for Wall Follower. I believe it's because my code doesn't quite rotate the robot correctly so that its side is parallel to the wall. I redid this code maybe 10-15 times, and unfortunately, I still wasn't able to fix this in time, so my Wall Follower doesn't quite work as the project specifies it should. It might be something that I'm fundamentally misunderstanding about the robot's sensor measurements or movement, too, but I can't articulate what that would be.

## Future Work
I almost certainly would first fix the Wall Follower so that it works per the project parameters if I had more time. If I could get the robot to at least drive from a fixed distance from the wall, the other issue with the choppy turns could be fixed in the same vein.
In terms of new additions, one feature I would smooth out for sure is the robot's turns in the Person Follower and Wall Follower. In both programs, the robot turns corners by first stopping, rotating the appropriate angle, and then resuming linear-x motion. While this is acceptable movement, it's choppy and looks less clean than if the turns were rounded out.  

## Takeaways
1. Since I didn't fully understand how the `/scan` topic recorded sensor data and what kinds of sensor data it recorded, it took me a while to make substantial progress on the Person Follower and Wall Follower. Instead of just guessing at the data points that it returned when I first started the project, I should've played around with the `/scan` echo and investigate the data that is actually returned. I would've figured out the features of the topic much more quickly, and my code could've gone through less unnecessary versions. In the future, I'll make sure to fiddle with topics or ROS features I'm not familiar with first before I attempt to use them.
2. This isn't fully code-related, but I definitely need to create a timeline for these projects. While the week of this project in particular had been much crazier than usual, my usual procedure of picking up the project when I have time every other day or so was not effective. In the future, I'll make sure to plan out the trajectory of my desired project progress. That way, I'm not in a situation where my program is buggy but I don't have quite enough time to iron out all of the kinks. I also should shift around my schedule so that I can attend more Office Hours, because my current schedule basically precludes me from 2/3 of the OH sessions, but I definitely need the help.