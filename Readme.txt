ENPM661 Project:2 By Kashif Ansari (120278280) & Abhishek Avhad (120257162)
-------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------

#Libraries used for part1:

>numpy
>matplotlib.pyplot
>cv2
>heapq
>time
>rclpy
>geometry_msgs.msg
>rclpy
>threading

#Steps to run the Part1 program:

1. To run the a_star.py use any code editor of your choice, VSC, Spyder or Colab is preffered. 
2. Run the program and close the window of inital_map to proceed ahead.
3. Input clearance (here it will apear blue in color)
4. Input robot radius (this will be added to clearance) and RPM1, RPM2.
5. Input start co-ordinates.
6. Input goal co-ordinates.
7. Please wait for path to reach the goal and an output will be backtracked.
8. Finally the scanned map will appear with the generated path. 


Github link: https: //gitfront.io/r/kashifansaricodes/StESCEbwXhVE/Path-planning-A-star-turtlebot3/
Drive link for videos: https://drive.google.com/drive/folders/1JF4Qo_PQuirh4qbkyVwyqzbXQG-Z3Qcy?usp=sharing

<<< Test Cases for Part 1 >>>

Test Case1:
-------------------------------
Enter the clearance value:10
Enter the RPM1 value:50
Enter the RPM2 value:100

Enter your start x-coordinate: 700
Enter your start y-coordinate: 1000
Enter start theta value: 30

Enter your goal x-coordinate: 3300
Enter your goal y-coordinate: 1300

Testcase2:
-------------------------------
Enter the clearance value:5
Enter the RPM1 value:25 
Enter the RPM2 value:100

Enter your start x-coordinate: 250
Enter your start y-coordinate: 250
Enter start theta value: 90

Enter your goal x-coordinate: 1000
Enter your goal y-coordinate: 1000

Testcase3:(Extreme Case)
-------------------------------
Enter the clearance value:10
Enter the RPM1 value:90
Enter the RPM2 value:100

Enter your start x-coordinate: 300
Enter your start y-coordinate: 260
Enter start theta value: 30

Enter your goal x-coordinate: 4800
Enter your goal y-coordinate: 1500


# For Part 2: Running Turtlebot waffle with ROS in Ubuntu 
(map is shrinked by a factor of 10)

Step 1: Unzip the package and paste it in your src folder
Step 2: Open two terminal windows and put the commands bellow line by line.

Terminal 1
1) cd ~/project3_ws
2) colcon build
3) source install/setup.bash
4) ros2 launch turtlebot3_project3 competition_world.launch.py

Terminal 2
1) cd ~/project3_ws
2) colcon build
3) source install/setup.bash
2) ros2 run turtlebot3_project3 a_star_pub.py


---------------------------
Fixing possible errors:
---------------------------

Incorrect library version loaded
Failed to initialize VideoWriter

run this command in terminal: 
pip install opencv-python --upgrade




