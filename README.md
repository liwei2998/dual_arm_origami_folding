# dual_arm_urdf
This file includes dual universal robots, robotiq 2f gripper, ft sensor, customized soft gripper, environment, and launch files to view them.
1. add this src folder to your src, catkin_make.
2. to view the whole effect, $ roslaunch ur_description view_ur10.launch

Problems faced in urdf and xacro:
prob kind a: cannot find file
1.$ .<path>/parser <path>my_robot.urdf
2.$touch <path>my_robot.xacro
3.go to <path>my_robot.xacro, check if new file (my_robot.xacro) generates. If yes, delete the old file. If not, nothing needs to do.
  
