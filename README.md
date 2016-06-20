# sdf_slam_2d
Please note that this is experimental code. 
Thus, it is neither stable nor does it follow sane programming guidelines.

Teaser video at https://www.youtube.com/watch?v=j1vs0sUXAQc



If you want to try:

git clone https://github.com/smARTLab-liv/sdf_slam_2d.git --branch=iros15

rosdep install sdf_slam

apt-get install ros-indigo stdr-simulator

apt-get install ros-indigo-teleop-twist-keyboard

catkin_make

roslaunch sdf_slam_2d local_stdr_sim.launch

roslaunch sdf_slam_2d local_sdf_slam_stdr.launch

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot0/cmd_vel

rosrun rviz rviz -d $(rospack find sdf_slam_2d)/cfg/sim.rviz