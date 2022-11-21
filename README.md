# turtlebot3 real-world brick search

## Description:
The goal of this assignment was to use a ‘turtlebot3’ mobile robot to find a red brick in a simulated maze environment. To achieve that goal, we had to implement concepts of localisation, navigation, SLAM, exploration and computer vision with Python in a ROS framework. In addition we tried to solve the task in a real-world setup. We were able to perform a 2D LiDAR SLAM while performing teleoperation in order to create a 2D occupancy map.

![grafik](https://user-images.githubusercontent.com/115760050/202964936-fb3ef972-04c3-4c22-8f64-00a0bada21bc.png)

Then we used the map to navigate the robot to hardcoded waypoints. After that we used the explore_lite package to perform autonomous exploration with the robot in an unknown environment. Finally we used the onboard camera of the robot to detect the red brick and indicate to the user that the brick was found while exploring the environment.

![grafik](https://user-images.githubusercontent.com/115760050/202965007-dc376408-e32b-48b5-a964-96744124da75.png)

real-world demonstration: https://youtu.be/moILBKuq8Ls


