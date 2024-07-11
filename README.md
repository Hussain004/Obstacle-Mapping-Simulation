**Obstacle Mapping in Gazebo using ROS1**

This ROS1 Python script demonstrates obstacle mapping using a Turtlebot3 robot in the Gazebo simulation environment.

**Key Features:**

- The robot moves forward until it detects an obstacle within a user-specified minimum distance.
- The robot then rotates to align its orientation with the direction of the nearest obstacle.
- The script maps the position of the detected obstacle in the world coordinate frame and plots it on a 2D graph.
- The resulting map is saved as a PNG file.

**Prerequisites**

- ROS1 (Melodic or Noetic)
- Turtlebot3 packages
- Python 3
- Matplotlib library

**Usage**

1. Install the required ROS packages and Matplotlib library.
1. Place the provided Python script in your ROS workspace's src directory.
1. Build the ROS workspace using catkin\_make or catkin build.
1. Run the script:

   Copy

   rosrun <your\_package\_name> obstacle\_mapping.py

1. When prompted, enter the minimum distance (in meters) that the robot should maintain from obstacles.
1. The robot will start moving and mapping the obstacles it detects. The resulting map will be saved as map.png in the current directory.

**Demonstration Video**

A demonstration video showcasing the obstacle mapping functionality can be found here.

**Code Explanation**

1. The script initializes a ROS node and subscribes to the /odom and /scan topics to obtain the robot's position and sensor data, respectively.
1. The move\_minimum\_distance() function moves the robot forward until it detects an obstacle within the specified minimum distance.
1. The rotate\_goal() function rotates the robot to align its orientation with the direction of the nearest obstacle.
1. The obj\_coor() function calculates the world coordinates of the detected obstacle based on the robot's position and the sensor data.
1. The map\_object() function continuously moves the robot forward, detects obstacles, and plots their positions on a 2D graph using Matplotlib.
1. Finally, the script saves the resulting map as a PNG file and displays it.

**Acknowledgments**

This project was developed as part of a ROS learning exercise. The Turtlebot3 package and Gazebo simulation environment were used to create the demo.

