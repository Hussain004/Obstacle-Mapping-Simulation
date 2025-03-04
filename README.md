﻿<h1> Obstacle Mapping in Gazebo using ROS1 </h1>

This ROS1 Python script demonstrates obstacle mapping using a Turtlebot3 robot in the Gazebo simulation environment.

![Shwocase](img1.png "turtlebot3-burger mapping an obstacle")


<h1> Key Features: </h1>

- The robot moves forward until it detects an obstacle within a user-specified minimum distance.
- The robot then rotates to align its orientation with the direction of the nearest obstacle.
- The script maps the position of the detected obstacle in the world coordinate frame and plots it on a 2D graph.
- The resulting map is saved as a PNG file.



<h1> Prerequisites </h1>

- ROS1 (Melodic or Noetic)
- Turtlebot3 packages
- Python 3
- Matplotlib library



<h1> Usage </h1>

1. Install the required ROS packages and Matplotlib library.
2. Place the provided Python script in your ROS workspace's src directory.
3. Build the ROS workspace using catkin\_make or catkin build.
4. Run the script:

   Copy

   rosrun <your\_package\_name> obstacle\_mapping.py

5. When prompted, enter the minimum distance (in meters) that the robot should maintain from obstacles.
6. The robot will start moving and mapping the obstacles it detects. The resulting map will be saved as map.png in the current directory.



<h1> Demonstration Video </h1>

A demonstration video showcasing the obstacle mapping functionality can be found in the repo (demo).



https://github.com/user-attachments/assets/ba29af62-06e4-4e86-8cce-553fcbb14031





<h1> Code Explanation </h1>

1. The script initializes a ROS node and subscribes to the /odom and /scan topics to obtain the robot's position and sensor data, respectively.
2. The move\_minimum\_distance() function moves the robot forward until it detects an obstacle within the specified minimum distance.
3. The rotate\_goal() function rotates the robot to align its orientation with the direction of the nearest obstacle.
4. The obj\_coor() function calculates the world coordinates of the detected obstacle based on the robot's position and the sensor data.
5. The map\_object() function continuously moves the robot forward, detects obstacles, and plots their positions on a 2D graph using Matplotlib.
6. Finally, the script saves the resulting map as a PNG file and displays it.



<h1> Acknowledgments </h1>

This project was developed as part of a ROS learning exercise. The Turtlebot3 package and Gazebo simulation environment were used to create the demo.

