# diff_robot
install the pigpio library using sent info


Then run commands in order

colcon build
source install/setup.sh
sudo pigpiod
ros2 launch icarus_pi launch_robot.launch.py 
