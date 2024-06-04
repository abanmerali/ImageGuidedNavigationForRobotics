# PathPlanning

The Path Planning module is designed for planning optimal electrode insertion trajectories using 3D Slicer. It computes the optimal paths for electrode insertions while avoiding critical structures and respecting user-defined constraints.

## Module Description

The module processes the following inputs:
- **Target and Entry Points**: Defined as `vtkMRMLMarkupsFiducialNode`, these represent the start (entry) and end (target) points for the insertion paths.
- **Target Volume**: A `vtkMRMLLabelMapVolumeNode` that encapsulates the target points.
- **Critical Volumes**: Optional `vtkMRMLLabelMapVolumeNode` volumes that should be avoided during path planning.
- **Full Volume**: An optional `vtkMRMLScalarVolumeNode` representing the entire volume within which insertions are to be made.

The module computes optimal paths ensuring:
- Entry into the target volume.
- Avoidance of critical structures.
- Adherence to a maximum user-defined path length.
- Optionally, adherence to a maximum insertion angle.

The most optimal path is then output back to the MRML Scene as a `vtkMRMLMarkupsFiducialNode`.

## Key Features

1. **Path Computation**: Calculates valid paths from entry points to target points while avoiding critical volumes and considering constraints such as maximum path length and insertion angle.
2. **Intersection Checks**: Uses `vtkProbeFilter` or 'vtkOBBTree` for intersection checks with critical volumes.
3. **Distance Mapping**: Utilises Signed Maurer Distance Map filter to compute minimum distances from paths to critical volumes.
4. **Angle Calculation**: Computes the angle between the trajectory and the normal of the surface at the intersection point within the full volume.
5. **Visualisation**: Outputs the computed optimal path as a `vtkMRMLModelNode` for visualisation.

## How to Use

1. **Load the Module in 3D Slicer**: Open Extension Wizard from the Developer Tools section and add the outermost PathPlanning file, select yes when asked if you would like to add the module. It will now be available under Surgical Planning.
2. **Set Inputs**: Select the full volume, target volume, critical volumes, target fiducials, and entry fiducials using the provided selectors.
3. **Configure Parameters**: Adjust the maximum length and maximum angle if needed. Optionally, enable the advanced options for more detailed checks of intersections.
4. **Generate Path**: Click the "Generate Path" button to compute the optimal path.
5. **Review Results**: The optimal path will be stored as target and entry points in the selected output fiducial node and a line in Optimal Path, a 'vtkPolyData'.
### OpenIGTLink for ROS Communication
6. **Install Extensions**: Open the Extensions Manager under the View tab, search for and install "SlicerIGT" and "SlicerOpenIGTLink".
7. **Create Connector**: Open OpenIGTLinkIF module under IGT. Click the first plus button to add a new connector, and select the Server and Active boxes. Note the port number and host.
8. **Select Data to Send**: Fully expand the connector in the I/O Configuration window to reveal "OUT". Select this and choose the output markup fiducial with the optimal path from the drop down. Select "Push on Connect" and wait to click send.
8. **Continue in ROS**: Continue to use OpenIGTLink in ROS to receive the entry and target points, return to send data when ready. Further documentation on OpenIGTLink in ROS can be found in this repository.

## Detailed Functionality

### Path Computation
The `process` function handles the core computation:
- Validates input data.
- Filters out invalid target points not within the target volume.
- Optionally filters out entry points that are within the full volume.
- Computes all valid paths between the target and entry points while avoiding critical structures.
- Filters optimal paths by selecting those furthest from critical volumes and with the smallest insertion angles.

### Intersection Checks
Two methods are provided for intersection checks:
- `checkIntersection`: Uses `vtkProbeFilter` to check for intersections along the path.
- `checkIntersectionOBB`: Uses `vtkOBBTree` for more detailed intersection checks when requested by the user.

### Distance Mapping
The `computeMinDistances` function computes the minimum distances from each path to the surfaces of critical volumes using the Signed Maurer Distance Map filter.

### Angle Calculation
The `intersectionAngle` function calculates the angle between the trajectory and the normal of the surface at the intersection point within the full volume.

## Dependencies

This module relies on the following libraries and functionalities:
- **3D Slicer**: The main framework within which the module operates.
- **VTK (Visualization Toolkit)**: For data processing and visualization tasks.
- **SimpleITK**: For image processing tasks.
- **sitkUtils**: For conversion between SimpleITK and VTK data structures.
- **ScriptedLoadableModule**: For slicer widget and UI interaction - https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
- **SlicerIGT**: A 3D Slicer extension for image-guided therapy research. Installed via the 3D Slicer Extensions Manager.
- **SlicerOpenIGTLink**: A 3D Slicer extension for OpenIGTLink communication. Installed via the 3D Slicer Extensions Manager.

## Acknowledgements

This module was developed by Aban Merali at King's College London. It was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab, and Steve Pieper, Isomics, Inc., partially funded by NIH grant 3P41RR013218-12S1. Special thanks to Dr. Rachel Sparks for guidance and support during the development of this module.

 
