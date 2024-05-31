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
2. **Intersection Checks**: Uses `vtkOBBTree` and `vtkProbeFilter` for detailed intersection checks with critical volumes.
3. **Distance Mapping**: Utilizes Signed Maurer Distance Map filter to compute minimum distances from paths to critical volumes.
4. **Angle Calculation**: Computes the angle between the trajectory and the normal of the surface at the intersection point within the full volume.
5. **Visualization**: Outputs the computed optimal path as a `vtkMRMLModelNode` for visualization.

## How to Use

1. **Load the Module**: Open the Path Planning module in 3D Slicer.
2. **Set Inputs**: Select the full volume, target volume, critical volumes, target fiducials, and entry fiducials using the provided selectors.
3. **Configure Parameters**: Adjust the maximum length and maximum angle if needed. Optionally, enable the advanced options for more detailed checks.
4. **Generate Path**: Click the "Generate Path" button to compute the optimal path.
5. **Review Results**: The optimal path will be stored in the selected output fiducial node and visualized in the 3D scene.

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
- `checkIntersectionOBB`: Uses `vtkOBBTree` for detailed intersection checks.
- `checkIntersection`: Uses `vtkProbeFilter` to check for intersections along the path.

### Distance Mapping
The `computeMinDistances` function computes the minimum distances from each path to the surfaces of critical volumes using the Signed Maurer Distance Map filter.

### Angle Calculation
The `intersectionAngle` function calculates the angle between the trajectory and the normal of the surface at the intersection point within the full volume.

## Dependencies

This module relies on several core libraries and functionalities provided by 3D Slicer, VTK, and SimpleITK.

## Acknowledgements

This module was developed by Aban Merali at King's College London. It was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab, and Steve Pieper, Isomics, Inc., partially funded by NIH grant 3P41RR013218-12S1. Special thanks to Dr. Rachel Sparks for guidance and support during the development of this module.

## License

This module is distributed under the Slicer License, Version 1.0. See the accompanying LICENSE file or visit [Slicer License](https://github.com/Slicer/Slicer/blob/main/License.txt) for more details.

 
