# Icarus 🪽

### Welcome to Ikaros Robotics' first venture, Project Icarus!

Icarus is an open-source project that students or hobbyists can recreate to gain experience with ROS, SLAM, and Nav2. It is also a way for us to share our love of robotics! Icarus is a four-wheeled, mecanum-drive robot capable of manual and autonomous navigation. This is the project's main repository and contains all of the information and code to create the physical robot. Our sub-repo ([Icarus Jetson Nano Repository](https://github.com/steelgit/icarus_nano)) contains all of the information required to add autonomous navigation to the robot and can be used without recreating Icarus.

<br />

Embed Video here

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

Some of our core components include:

- Jetson Nano
- Raspberry Pi 4
- Motors with Quadrature Encoders
- 2-D LiDAR
- Raspberry Pi Camera V2

<br />

A full list of our materials can be found on our [Materials List ](https://github.com/steelgit/icarus_pi/blob/daedalus_main/docs/Materials%20List.pdf).

<br />
<br />

# Electrical

Electrical Summary.

<br />

#### Electrical Diagram

![Icarus Electrical Diagram](/docs/Electrical/Icarus%20Electrical%20Diagram.png "Icarus Electrical Diagram")

<br />

A more in-depth description of our electrical design, as well as electrical schematics and pinout descriptions, can be found in our [Electrical Documentation](https://github.com/steelgit/icarus_pi/blob/daedalus_main/docs/Electrical/Electrical%20README.md).

<br />
<br />

# Mechanical

Mechanical Summary.

<br />

#### Mechanical Diagram

<br />

A more in-depth description of our mechanical design, as well as CAD files, can be found in our [Mechanical Documentation](https://github.com/steelgit/icarus_pi/blob/daedalus_main/docs/Mechanical/Mechanical%20README.md).

<br />
<br />

# Helpful Resources

1. 
