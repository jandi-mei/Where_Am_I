# Where Am I? Project

## Project Overview
The "Where Am I?" project focuses on robot localization using the Adaptive Monte Carlo Localization (AMCL) algorithm within a simulated environment. The goal of this project is to enable a robot to accurately determine its position and orientation in a map while navigating through a given environment. This project integrates various ROS packages and tools to achieve reliable localization.

## What Does It Do?
This project allows a robot to:
- Localize itself within a predefined map using sensor data.
- Navigate through the environment while maintaining accurate position estimates.
- Utilize RViz for visualization of the robot's position and the map.

## Packages and Tools Used
- **ROS (Robot Operating System)**: A flexible framework for writing robot software.
- **AMCL (Adaptive Monte Carlo Localization)**: An algorithm used for robot localization.
- **Gazebo**: A simulation tool that provides a realistic environment for testing robot navigation.
- **RViz**: A 3D visualization tool for ROS that allows users to visualize the robot's state and sensor data.
- **Map Server**: A ROS node that provides the map to the localization algorithm.
- **Move Base**: A ROS package that provides navigation capabilities.

## Project Structure
project_directory/ 

    ├── launch/ 
    │ ├── amcl.launch 
    │ ├── map_server.launch 
    │ └── move_base.launch 
    ├── maps/ 
    │ └── my_map.yaml 
    ├── world/ 
    │ └── my_world.world └
    ── src/ 
    ├── localization_node.cpp 
    └── robot_control.cpp


## Steps to Run the Project

1. **Install Dependencies**:
   Ensure that you have ROS and Gazebo installed on your system. Follow the installation instructions from the [ROS website](http://wiki.ros.org/ROS/Installation) and the [Gazebo website](http://gazebosim.org/).

2. **Clone the Repository**:
   Clone your project repository to your local machine:
   ```bash
   git clone <repository_url>
   cd project_directory

3. **Build the Project**: Navigate to the project directory and build the project using catkin_make:

  bash

    cd ~/catkin_ws
    catkin_make

4. **Source the Workspace**: Source the workspace to ensure ROS can find your packages:

bash

    source devel/setup.bash

5. **Launch the Simulation**: Launch the Gazebo simulation with your world:

        roslaunch world/my_world.launch

6. **Run the Localization Nodes**: In a new terminal, run the AMCL and Move Base nodes:

        roslaunch launch/amcl.launch
        roslaunch launch/move_base.launch
    
7. **Visualize in RViz**: Open RViz to visualize the robot's localization and the map:

        rosrun rviz rviz


**Conclusion**

The "Where Am I?" project demonstrates the integration of localization algorithms and simulation tools to enable a robot to navigate and understand its position in an environment. This project serves as a practical application of robotics concepts and prepares me for more advanced robotics challenges.


Make sure to replace `<repository_url>` with the actual URL of your project repository.
