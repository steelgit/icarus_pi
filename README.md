# Icarus 🪽

### Welcome to Ikaros Robotics' first venture, Project Icarus!

Icarus is an open-source project that students or hobbyists can recreate to gain experience with ROS, SLAM, and Nav2. It is also a way for us to share our love of robotics! Icarus is a four-wheeled, mecanum-drive robot capable of manual and autonomous navigation. This is the project's main repository and contains all of the information and code to create the physical robot. Our sub-repo ([Icarus Jetson Nano Repository](https://github.com/steelgit/icarus_nano)) contains all of the information required to add autonomous navigation to the robot and can be used without recreating Icarus.

<br />

![Icarus Video](/docs/Icarus_Video.gif "Video of Icarus Operating Autonomously")

<br />
<br />

# Initial Run

>[Install the pigpio library](https://abyz.me.uk/rpi/pigpio/download.html)

>To install all of the required dependencies from the package.xml run:

    rosdep update --include-eol-distros    
<br />

    rosdep install --from-paths src -y --ignore-src --rosdistro foxy

<br />
<br />

# To Launch Icarus

>On startup, launch the PiGPIO Daemon using:

    sudo pigpiod

>Launch the robot!

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
- Mecanum Wheels

<br />

A full list of our materials can be found on our [Materials List ](/docs/Materials%20List.pdf).

<br />
<br />

# Mechanical

The design of this robot was inspired by the turtlebot-burger educational platform. This design allows for a high degree of flexibility in the placement of components. Additionally, due to the modular design philosophy here, its possible to add, remove, or swap out new parts and sensors as you see fit. If desired, you can create your own custom fixtures and components to mount onto your robot as you see fit.
The mounting holes on the base plates have an initial diameter of 4.5mm in order for m3 flathead screws to sit flush within the plate, and then we have a 3.5mm diameter hole that's also designed for m3 screws; with a center-to-center distance of 20mm between each hole.
for a more vibration-resistant fit, add in threaded brass inserts to your mounting components by melting them in via soldering iron.

<br />

#### Mechanical Assembly

![Icarus Mechanical Design](/docs/Mechanical/Mechanical%20Assembly.png "Icarus Mechanical Design")

<br />

A more in-depth description of our mechanical design, as well as our STL files, can be found in our [Mechanical Documentation](/docs/Mechanical/Mechanical%20README.md).

<br />
<br />

# Electrical

❗Electrical Summary.

<br />

#### Electrical Diagram

![Icarus Electrical Diagram](/docs/Electrical/Icarus%20Electrical%20Diagram.png "Icarus Electrical Diagram")

<br />

A more in-depth description of our electrical design, as well as our electrical schematic and pinout descriptions, can be found in our [Electrical Documentation](/docs/Electrical/Electrical%20README.md).

<br />
<br />

# Mecanum Wheels

❗Mecanum wheel summary.

<br />

#### Mecanum Wheel Model

❗embed a mecanum wheel cad model pic or kinematics drawing
![alt text](image.png)
<br />

A more in-depth description of the various software challenges we faced with mecanum wheels can be found in our [Mecanum Wheel Documentation](/docs/Mecanum%20Wheel%20README.md).

<br />
<br />

# Lessons Learned & Future Improvements

❗Overall blurb

<br />

### Software

❗blurb

<br />

### Electrical

The switches were a nice addition, however, they could be removed to simplify the power and ground connections. Since the battery connectors are easily accessible, the system can be quickly de-energized without switches by simply disconnecting the connectors. There were also a couple of instances where wires had to be spliced 3 or more times and soldering these splices was a little finicky given 

<br />

### Mechanical

It was never a wise decision to design something with the idea that we would never move the component from its position, or that it would need to be removed from its mounting area. Always design with ease of assembly and disassembly in mind

Make use of not only the horizontal space available on a given surface. If its more convenient for assembly, create designs that take advantage of the vertical space given.
i.e: moutning this vertically, upside down, stacked, etc.
<br />
<br />

# References

- [ROS2 Foxy Official Documentation](https://docs.ros.org/en/foxy/index.html)
- [The PiGPIO Library](https://abyz.me.uk/rpi/pigpio/index.html)
- [Articulated Robotics - Build a Mobile Robot with ROS](https://articulatedrobotics.xyz/tutorials/mobile-robot/project-overview)
- [ECAM Eurobot - Mecanum Wheels](https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html)
- [Turtlebot3](https://www.turtlebot.com/turtlebot3/)
