# CatkinWorkspaceforROS
## For robotic simulation and IGTL-Bridge communication

These instructions and the scripts in this workspace ROS components required to translate planned trajectories from 3D Slicer into actionable movements by a robotic arm. This involves defining a robot using a Unified Robot Description Format (URDF) file, using MoveIt for planning and execution, and employing RViz for visualisation. The system subscribes to data sent from 3D Slicer using 'ROS_IGTL_BRIDGE' and publishes the resulting joint states for the defined robot.

## Workflow Description

This workflow uses and processes the following inputs:
- **Trajectory Points**: Received as `igtlpoint` messages from 3D Slicer, representing the start (entry) and end (target) points for the robot's end-effector path.
- **Robot Description**: Defined using a URDF file that describes the robot's physical configuration, including links, joints, and their properties.
- **Planning and Execution**: Utilises MoveIt for motion planning and execution of the robot's movements along the received trajectory points.

The module ensures:
- Accurate transformation of received points from millimeters to meters.
- Safe and precise movement of the robotic arm to the target points.
- Adherence to the robot's joint limits and avoidance of collisions.

The received trajectory points are visualised in RViz to monitor the robot's movements in real-time.

## Key Features

1. **ROS-IGTL Bridge**: Facilitates communication between 3D Slicer and ROS using the 'ROS-IGTL-Bridge' package to receive and process trajectory point. This may also be used to senf points back to slicer for further validation.
2. **Point Transformation**: 'my_controller.py' (part of the 'moveit_6dof' package) subscribes to the ROS topic '/IGTL_POINT_IN' to receive entry and target points. Converts received trajectory points from millimetres to metres to ensure compatibility with ROS and MoveIt.
3. **Trajectory Execution**: Uses MoveIt to compute valid joint trajectories and execute the robot's movements along the planned path, using MoveIt's 'MoveGroupCommander'.
4. **Visualisation**: Utilises RViz to visualise the robot's URDF model, planned trajectories, and real-time movements.

## How to Use
### Terminal Commands

1. **Setup ROS Workspace**: 
    ```console
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```
2. **Begin RViz Visualisation**: 
    ```console
    roslaunch moveit_6dof demo.launch
    ```
3. **Run Controller**: 
    In a separate terminal:
    ```console
    roscore
    ```
    In a separate terminal:
    ```console
    chmod +x ~/src/moveit_6DOF/my_robot_controller.py
    rosrun moveit_6DOF my_robot_controller.py
    ```

## Detailed Functionality

### ROS-IGTL Bridge
The ROS-IGTL Bridge is responsible for establishing communication between 3D Slicer and ROS. This bridge facilitates the  transmission of trajectory points from 3D Slicer to ROS, allowing the robotic arm to execute the path based on medical imaging data.

### Robot Controller
The my_robot_controller.py script handles the execution of the robot's movements based on the received trajectory points. The process is as follows:
- **Initialisation**: The PointMover class initialises the ROS node, MoveIt Commander, and the robot's move group.
- **Subscription**: The script subscribes to the '/IGTL_POINT_IN' topic to receive 'igtlpoint' messages containing the trajectory points.
- **Point Conversion**: The received points are converted from millimetres to metres to match the units used by ROS and MoveIt.
- **Waypoint Storage**: The converted points are stored as waypoints for the robot to follow.
- **Cartesian Path Planning**: The script uses MoveIt's MoveGroupCommander to compute a Cartesian path connecting the waypoints.
- **Execution Check**: The script checks if the Cartesian path planning was successful. If the fraction of the path planned is 1.0 (i.e., 100% of the path was planned), the robot executes the plan.
- **Movement Execution**: The robot moves to the target points along the planned path. The script ensures any residual movement is stopped and clears the pose targets to prepare for the next set of points.

### Visualisation
RViz is used to visualise the robot's URDF model, planned trajectories, and real-time movements. This allows users to monitor the robot's actions and verify the correctness of the planned paths. The process includes:
- **Loading the URDF Model**:  The robot's URDF file is loaded into RViz to display the robot model.
- **Trajectory Display**: The planned trajectories are visualized in RViz, showing the path the robot will follow.

## Dependencies

This framework and its packages rely on the following libraries and functionalities:

- **ROS Noetic**: The primary framework for robotic software development - http://wiki.ros.org/noetic/Installation/Ubuntu
- **MoveIt**: A motion planning framework used for robotic path planning and execution. It includes moveit_commander for Python interface and MoveGroupCommander for motion planning.
- **rospy**: ROS client library for Python, used for ROS node creation and communication. Included with ROS.
- **geometry_msgs**: ROS message types for geometric primitives like points and poses. Included with ROS.
- **ROS-IGTL-Bridge**: Facilitates communication between ROS and OpenIGTLink. Installation instructions can be found at ROS-IGTL-Bridge repository.
- **catkin**: The build system for ROS packages, installed with ROS and used for building the workspace.

## Acknowledgements

This framework and its packages were developed by Aban Merali at King's College London. Special thanks to Dr. Rachel Sparks for guidance and support during the development of this module.

 
