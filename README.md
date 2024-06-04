# ImageGuidedNavigationForRobotics
## Robotic Electrode Insertion Planning Suite

The ImageGuidedNavigationForRobotics suite integrates 3D Slicer and ROS components to plan and execute optimal electrode insertion trajectories for robotic systems. This platform leverages advanced image processing and robotic simulation techniques to ensure precise and safe surgical interventions.

## Overview

This suite consists of two main components:
1. **PathPlanning**: A 3D Slicer module designed for planning optimal electrode insertion trajectories - https://github.com/abanmerali/ImageGuidedNavigationForRobotics/blob/main/PathPlanning/README.md
2. **CatkinWorkspaceforROS**: A ROS workspace for robotic simulation and IGTL-Bridge communication to execute the planned trajectories - https://github.com/abanmerali/ImageGuidedNavigationForRobotics/blob/main/ROS/README.md

## PathPlanning

The Path Planning module in 3D Slicer computes the optimal paths for electrode insertions while avoiding critical structures and respecting user-defined constraints. It processes inputs such as target and entry points, target volumes, and optional critical volumes, and outputs the most optimal path to the MRML Scene.

### Key Features
- **Path Computation**: Calculates valid paths from entry points to target points while considering constraints.
- **Intersection Checks**: Uses advanced methods to ensure paths avoid critical volumes.
- **Distance Mapping**: Computes minimum distances from paths to critical volumes.
- **Angle Calculation**: Calculates the angle between the trajectory and the normal of the surface.
- **Visualisation**: Outputs the computed optimal path for visualisation.

For detailed instructions and usage, refer to the [PathPlanning README](ImageGuidedNavigationForRobotics/ROS/README.md).

## CatkinWorkspaceforROS

The ROS components translate the planned trajectories from 3D Slicer into actionable movements by a robotic arm. This involves defining a robot using a URDF file, using MoveIt for planning and execution, and employing RViz for visualisation. The system subscribes to data sent from 3D Slicer using 'ROS_IGTL_BRIDGE' and publishes the resulting joint states for the defined robot.

### Key Features
- **ROS-IGTL Bridge**: Facilitates communication between 3D Slicer and ROS.
- **Point Transformation**: Converts received trajectory points for compatibility with ROS and MoveIt.
- **Trajectory Execution**: Uses MoveIt for computing and executing valid joint trajectories.
- **Visualisation**: Utilises RViz to monitor the robot's movements in real-time.

For detailed instructions and usage, refer to the [ROS README](ImageGuidedNavigationForRobotics/PathPlanning/README.md).

## Dependencies

This suite relies on the following libraries and functionalities:
- **3D Slicer**: The main framework for the PathPlanning module.
- **VTK (Visualization Toolkit)**: For data processing and visualisation tasks.
- **SimpleITK**: For image processing tasks.
- **sitkUtils**: For conversion between SimpleITK and VTK data structures.
- **ScriptedLoadableModule**: For slicer widget and UI interaction.
- **SlicerIGT**: A 3D Slicer extension for image-guided therapy research.
- **SlicerOpenIGTLink**: A 3D Slicer extension for OpenIGTLink communication.
- **ROS Noetic**: The primary framework for robotic software development.
- **MoveIt**: For robotic path planning and execution.
- **rospy**: ROS client library for Python.
- **geometry_msgs**: ROS message types for geometric primitives.
- **ROS-IGTL-Bridge**: Facilitates communication between ROS and OpenIGTLink.
- **catkin**: The build system for ROS packages.

## Acknowledgements

This suite was developed by Aban Merali at King's College London. Special thanks to Dr. Rachel Sparks for guidance and support during the development of this module.

 
