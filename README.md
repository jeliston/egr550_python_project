# egr598_python_project

Youtube link : https://youtu.be/25unwgreXMU

Tested using Ubuntu 18.04 and ROS Melodic

To recrete it,

open terminal

mkdir catkin_ws

cd catkin_ws

mkdir src

git clone https://github.com/jeliston/egr550_python_project.git

unzip the file projetc_ws

in terminal

cd ..

catkin_make


Once that is done

paste the following in different terminals (dont forget to source each terminal ie source devel/setup.bash)

roslaunch xarm_gazebo xarm6_challenge.launch

roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch

rosrun path_planner planning.py

rosrun pick_place pick_place.py



# Referance
https://github.com/Manchester-Robotics/ManipulatorsFeb2022-Students/tree/main/04_03_2022/challenge
