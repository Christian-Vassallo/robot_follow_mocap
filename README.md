robot_follow_mocap
==================

The following script allows the user to control an holonomic robot through MoCap feedback.
The script works with the Turtlebot and Robulab10.
The Turtlebot is controlled by ROS Groovy (or Hydro): http://wiki.ros.org/Robots/TurtleBot
The Robulab10 is controlled using UDP connection and sending low level message of control.
Reference Robulab: http://www.doc-center.robosoft.com/@api/deki/files/3847/=RobuBOX.Services.Interop.DifferentialDriveUDPServer.pdf

Two different Motion Capture System (MoCap) have been tested: Motion Analysis (Cortex) and Vicon (Blade) system.

Reference Motion Analysis - evart_bridge: http://wiki.ros.org/evart_bridge/Tutorials
Reference Vicon - vicon_bridge: http://wiki.ros.org/vicon_bridge

Source file:
- mocapmessenger.hpp : establish connection with the MoCap system and provide function to acquire the 6D information of the object
- turtlebotmotionclass.hpp: provides method to control the Turtlebot in velocity
- Robulab10Class.hpp : provides methods to establish and control in velocity the Robulab10 robot
- trajectoryClass.hpp : provides methods to define the trajctory to follow

The control is based on following path theory.

 
