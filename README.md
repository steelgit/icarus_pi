# Icarus 🪽

### Welcome to Ikaros Robotics' first venture, Project Icarus!

Icarus is an open-source project that students or hobbyists can recreate to gain experience with ROS, SLAM, and Nav2. It is also a way for us to share our love of robotics! Icarus is a four-wheeled, mecanum-drive robot capable of manual and autonomous navigation. This is the project's main repository and contains all of the information and code to create the physical robot. Our sub-repo ([Icarus Jetson Nano Repository](https://github.com/steelgit/icarus_nano)) contains all of the information needed to add autonomous navigation to the robot and can be used independently of Icarus.


<br />
<br />

# Initial Run

>To install all of the required dependencies:

    rosdep install --from-paths src -y --ignore-src

>To install PiGPIO-d

    install the pigpio library using sent info

<br />
<br />

# To Launch Icarus
    
    sudo pigpiod
    ros2 launch icarus_pi launch_robot.launch.py 

<br />
<br />

# To Launch Icarus in Simulation

>With Planar Controller:

    ros2 launch icarus_pi launch_sim_planar.launch.py 

>With Mecanum Controller:

    ros2 launch icarus_pi launch_sim_mecanum.launch.py 

<br />
<br />

# Materials List

<br />
<br />

# Electrical Schematic

<br />
<br />

# Helpful Resources

1. 
