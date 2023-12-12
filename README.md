# Ros2_turtlesim
A ros2 task to play around with a turtlesim bot.

This project has been made for my ros2 course.

koch.py is a script that makes a koch snowflake.

constructor:

Publisher initialization to controll the turtle's movement
Subscription initialization to a topic that provides Pose messages
set_pen:

Create a client for the SetPen service
Wait for the service to become available
Call the SetPen service asynchronously
Check the result of the service call
go_straight:

Is a basic fuctions to controll the turtle that we learned in the class
turn:

Is a basic fuctions to controll the turtle that we learned in the class
set_spawnpoint:

Is a function that can set the turtle's starting position
Sets the pen
Wait for the pose information
Calculate Angle and Distance to Target Point
Execute Movement
Set the pen attributes
draw_koch:

Is a recursive function
The parameters: speed, omega, counter, distance
If the counter is 0 than the turtle go straight
Else 3 recursive function and turn function call with different omega values (koch snowflake requires 60, -120, 60 degree turns)
