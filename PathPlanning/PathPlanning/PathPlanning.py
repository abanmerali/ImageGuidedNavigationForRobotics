import logging
import warnings
import time
import os
from typing import Annotated, Optional, List
import numpy as np

import vtk

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode
from slicer import vtkMRMLLabelMapVolumeNode
from slicer import vtkMRMLMarkupsFiducialNode
# adding in a few libraries to help me out
import SimpleITK as sitk
import sitkUtils 

"""
PathPlanning: In this module I am going to take in target and entry points (as vtkMRMLMarkupsFiducialNodes) for electrode insertions, 
the target volume of these insertions will be taken in as a vtkMRMLLabelMapVolumeNode, and optionally the full insertion volume and 
critical structures to avoid. Optimal paths will be computed ensuring entry into the target volume and avoidance of critical structures.
Paths will be further narrowed by a maximum user-defined length and optional maximum insertion angle. The most optimal path will be 
output back to the MRML Scene as a vtkMRMLMarkupsFiducialNode.
"""


class PathPlanning(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("Path Planning") 
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Surgical Planning")]
        self.parent.dependencies = []  
        self.parent.contributors = ["Aban Merali (King's College London)"]
        self.parent.helpText = _("""
This is a module to compute optimal electrode insertion angles, using target and entry points given in vtkMRMLMarkupsFiducialNodes,
and volumes given as vtkMRMLLabelMapVolumeNodes denoting target and critical structures.
""")
        self.parent.acknowledgementText = _("""
The base of this file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1. Aban Merali has modified this, 
for part of Image-guide Navigation for Robotics taught through King's College London. Thank you to Dr Rachel Sparks for,
guiding and informing the devlopment of this 3D slicer module.
""")



#
# PathPlanningParameterNode
#


@parameterNodeWrapper
class PathPlanningParameterNode:
    """
    The parameters needed by module.

    fullVolume - The full volume that insertion happens within.
    targetVolume - The volume encasing the target points.
    criticalVolumes - The volumes to avoid during path planning.
    targetFiducial - Tip target points (path ends).
    entryFiducial - Insertion entry points (path starts).
    maximumLength - Upper limit for accepted trajectory length 
    pathFiducial - pairs with each target point and closest valid entry point (where angle of insertion is less than 55deg to cortex)
    maximumAngle - Maximum accepted angle deviation from the normal of fullVolume.
    checkInsertionAngle - If True, insertion angle is checked against max limit.
    detailedIntersectionCheck - If True, more detailed intersection check is performed with vtkOBBTree. 
    checkEntryFiducial - If True, entry points are checked to ensure they are not within insertion volume.
    """

    fullVolume: vtkMRMLScalarVolumeNode = None
    targetVolume: vtkMRMLLabelMapVolumeNode 
    criticalVolumes: List[vtkMRMLLabelMapVolumeNode] = []
    targetFiducial: vtkMRMLMarkupsFiducialNode
    entryFiducial: vtkMRMLMarkupsFiducialNode
    maximumLength: Annotated[float, WithinRange(0, 250)] = 100
    pathFiducial: vtkMRMLMarkupsFiducialNode
    maximumAngle: Annotated[float, WithinRange(0, 90)] = 55.0
    checkInsertionAngle: bool = False
    detailedIntersectionCheck: bool = False
    checkEntryFiducial: bool = False


#
# PathPlanningWidget
#


class PathPlanningWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None

    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)

        # Load widget from .ui file (created by Qt Designer).
        # Additional widgets can be instantiated manually and added to self.layout.
        uiWidget = slicer.util.loadUI(self.resourcePath("UI/PathPlanning.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)

        # Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
        # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
        # "setMRMLScene(vtkMRMLScene*)" slot.
        uiWidget.setMRMLScene(slicer.mrmlScene)

        # Create logic class. Logic implements all computations that should be possible to run
        # in batch mode, without a graphical user interface.
        self.logic = PathPlanningLogic()

        # Connections

        # These connections ensure that we update parameter node when scene is closed
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)

        # Buttons
        self.ui.applyButton.connect("clicked(bool)", self.onApplyButton)

        # Make sure parameter node is initialized (needed for module reload)
        self.initializeParameterNode()


    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        if hasattr(self, '_observerTag'):
            self.removeObservers()

    def enter(self) -> None:
        """Called each time the user opens this module."""
        # Make sure parameter node exists and observed
        self.initializeParameterNode()

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        if self._parameterNode:
            if hasattr(self, '_parameterNodeGuiTag') and self._parameterNodeGuiTag:
                self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
                self._parameterNodeGuiTag = None
            if hasattr(self, '_observerTag'):
                self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        # Parameter node will be reset, do not use it anymore
        self.setParameterNode(None)

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        # If this module is shown while the scene is closed then recreate a new parameter node immediately
        if self.parent.isEntered:
            self.initializeParameterNode()

    def initializeParameterNode(self) -> None:
        """Ensure parameter node exists and observed."""
        # Parameter node stores all user choices in parameter values, node selections, etc.
        # so that when the scene is saved and reloaded, these settings are restored.

        # Initialising the parameter node inside the try-except is to suppress the redundant error,
        # caused by CheckableNode widget (for selecting multiple critical volumes)
        try:
            self.setParameterNode(self.logic.getParameterNode())
        except RuntimeError as e:
            if "Unable to create GUI connector" in str(e):
                pass
            else:
                raise

        # Select default input nodes if nothing is selected yet to save a few clicks for the user
        if not self._parameterNode.fullVolume:
            firstVolumeNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLScalarVolumeNode")
            if firstVolumeNode:
                self._parameterNode.fullVolume = firstVolumeNode

    def setParameterNode(self, inputParameterNode: Optional[PathPlanningParameterNode]) -> None:
        """Set and observe parameter node, ensuring the GUI is updated when the parameter node is changed.
        """

        if self._parameterNode:
            if hasattr(self, '_parameterNodeGuiTag') and self._parameterNodeGuiTag:
                self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            if hasattr(self, '_observerTag'):
                self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            # Note: in the .ui file, a Qt dynamic property called "SlicerParameterName" is set on each
            # ui element that needs connection.
            self._checkCanApply()
            self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
            self._checkCanApply()

    def _checkCanApply(self, caller=None, event=None) -> None:
        if self._parameterNode:
            self.ui.applyButton.toolTip = _("Compute optimal path")
            self.ui.applyButton.enabled = True
        else:
            self.ui.applyButton.toolTip = _("Select input and output nodes")
            self.ui.applyButton.enabled = False

    def onApplyButton(self) -> None:
        """Run processing when user clicks "Apply" button."""
        with slicer.util.tryWithErrorDisplay(_("Failed to compute results."), waitCursor=True):
            # Perform path-finding process
            self.logic.process(self.ui.fullVolumeSelector.currentNode(), self.ui.targetVolumeSelector.currentNode(), 
                               self.ui.criticalVolumeSelector.checkedNodes(), self.ui.targetFiducialSelector.currentNode(), 
                               self.ui.entryFiducialSelector.currentNode(), self.ui.lengthThresholdSliderWidget.value,
                               self.ui.pathFiducialSelector.currentNode(), self.ui.angleThresholdSliderWidget.value,
                               self.ui.checkInsertionAngleCheckBox.isChecked(), self.ui.detailedIntersectionCheckBox.isChecked(),
                               self.ui.checkEntriesCheckBox.isChecked())


#
# PathPlanningLogic
#

class PathPlanningLogic(ScriptedLoadableModuleLogic):
    """
    This class implements the core computation for the PathPlanning module.
    It handles all logic operations required for computing valid trajectories between 
    target and entry points, avoiding critical volumes, and considering constraints 
    such as maximum path length and insertion angle.

    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        ScriptedLoadableModuleLogic.__init__(self)

    def getParameterNode(self):
        return PathPlanningParameterNode(super().getParameterNode())

    def hasImageData(self, volumeNode):
        """Returns true if the passed in volume node has valid image data.
        """
        if not volumeNode:
            logging.debug('hasImageData failed: no volume node')
            return False
        if volumeNode.GetImageData() is None:
            logging.debug('hasImageData failed: no image data in volume node')
            return False
        return True
    
    def hasFiducials(self, fiducialNode):
        """Returns true if the passed in fiducial contains at least 1 control point
        """
        if not fiducialNode:
            logging.debug('hasFiducials failed: no fiducial node')
            return False
        if fiducialNode.GetNumberOfControlPoints() < 1:
            logging.debug('hasFiducials failed: no fiducials in fiducial node')
            return False
        return True

    def isValidInputOutputData(self, inputTargetVolumeNode, inputTargetFiducialsNode, inputEntryFiducialsNodes, outputFiducialsNode):
      """Validates if inputs and outputs have been set and that they are not the same as input
      """
      if not inputTargetVolumeNode:
        logging.debug('isValidInputOutputData failed: no input target volume node defined')
        return False
      if not inputTargetFiducialsNode:
        logging.debug('isValidInputOutputData failed: no input target fiducials node defined')
        return False
      if not inputEntryFiducialsNodes:
        logging.debug('isValidInputOutputData failed: no input entry fiducials node defined')
        return False
      if not outputFiducialsNode:
        logging.debug('isValidInputOutputData failed: no output fiducials node defined')
        return False
      if inputTargetFiducialsNode.GetID()==outputFiducialsNode.GetID():
        logging.debug('isValidInputOutputData failed: input and output fiducial nodes are the same. Create a new output to avoid this error.')
        return False
      return True
    
    def visualisePath(self, entryPos, targetPos):
        """Outputs the path from the given points as a vtkPolyData for visualisation
        """
        # TESTING - visualise
        trajectory = vtk.vtkPolyData()
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()

        targetIndex = points.InsertNextPoint(targetPos)
        entryIndex = points.InsertNextPoint(entryPos)

        # Create a line between these two points
        line = vtk.vtkLine()
        line.GetPointIds().SetId(0, entryIndex)
        line.GetPointIds().SetId(1, targetIndex)
        lines.InsertNextCell(line)

        trajectory.SetPoints(points)
        trajectory.SetLines(lines)

        modelNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode")
        modelNode.SetName('Optimal Path')
        modelNode.SetAndObservePolyData(trajectory)

    def process(self,
                fullVolume: vtkMRMLScalarVolumeNode,
                targetVolume: vtkMRMLLabelMapVolumeNode,
                criticalVolumes: List[vtkMRMLLabelMapVolumeNode],
                targetFiducial: vtkMRMLMarkupsFiducialNode,
                entryFiducial: vtkMRMLMarkupsFiducialNode,
                maximumLength: Annotated[float, WithinRange(0, 250)],
                pathFiducial: vtkMRMLMarkupsFiducialNode,
                maximumAngle: Annotated[float, WithinRange(0, 90)] = 55.0,
                checkInsertionAngle: bool = False,
                detailedIntersectionCheck: bool = False,
                checkEntryFiducial: bool = False) -> None:
        """
        Compute trajectories and select optimal valid option.

        :param fullVolume - The full volume that insertion happens within.
        :param targetVolume - The volume encasing the target points.
        :param criticalVolumes - The volumes to avoid during path planning.
        :param targetFiducial - Tip target points (path ends).
        :param entryFiducial - Insertion entry points (path starts).
        :param maximumLength - Upper limit for accepted trajectory length. 
        :param pathFiducial - Output optimal pair of closest valid target and entry point. 
        :param maximumAngle - Maximum accepted angle deviation from the normal of fullVolume.
        :param checkInsertionAngle - If True, insertion angle is checked against max limit.
        :param detailedIntersectionCheck - If True, more detailed intersection check is performed with vtkOBBTree. 
        :param checkEntryFiducial - If True, entry points are checked to ensure they are not within insertion volume.
        """

        # For speed evaluation:
        startTime = time.time()
        logging.info("Processing started")

        # Validate input data
        if not self.isValidInputOutputData(targetVolume, targetFiducial, entryFiducial, pathFiducial):
            slicer.util.errorDisplay('Not all inputs and outputs are set.')
            return False
        if not (self.hasImageData(targetVolume) or self.hasImageData(fullVolume)):
            raise ValueError("Input target volume is not appropriatelly defined.")
        if (fullVolume is not None) and (not self.hasImageData(fullVolume)):
            raise ValueError("Input insertion (full) volume is not appropriatelly defined.")
        if not (self.hasFiducials(targetFiducial) or self.hasFiducials(entryFiducial)):
            raise ValueError("Input target or entry fiducials are not appropriatelly defined.")

        # Filter out invalid target points not within target volume
        validTargetFiducial = vtkMRMLMarkupsFiducialNode()
        pathProcessor = PathPointProcessor(checkInsertionAngle,detailedIntersectionCheck)
        pathProcessor.cleanPoints(targetFiducial, validTargetFiducial, targetVolume, criticalVolumes)
        # Optionally filter out entry points which are within the insertion volume
        if checkEntryFiducial and fullVolume:
            validEntryFiducial = vtkMRMLMarkupsFiducialNode()
            pathProcessor.cleanPoints(entryFiducial, validEntryFiducial, exclusionVolumes=[fullVolume])
        else:
            validEntryFiducial = entryFiducial

        # Compute all valid paths, below the and length and that do not intersect with critical structures if given
        validPaths = vtk.vtkPolyData()
        pathProcessor.computePaths(validTargetFiducial,validEntryFiducial,validPaths,criticalVolumes,fullVolume,maximumLength,maximumAngle)
        numPaths = validPaths.GetNumberOfCells()

        # Check if there are no valid paths
        if numPaths < 1:
            logging.debug('computePaths failed: no valid paths found between given target and entry points with given constraints')
            slicer.util.errorDisplay('No valid paths found, please check parameters and input data.')
            return False

        # Filter optimal paths sequentially with each condition
        optimalPathIndexes = np.arange(0,numPaths)

        # Choose paths furthest from the critical volumes
        if criticalVolumes:
            pathDistancesToVolumes = pathProcessor.computeMinDistances(validPaths,criticalVolumes)
            maxDist = pathDistancesToVolumes.max()

            # Check if there are no valid paths
            if maxDist == 0 and detailedIntersectionCheck:
                logging.debug('computeDistances found unexpected intersections: intersection of all trajectories with critical volumes found on coarser second distance check, insertion paths may be too close to critical structure or intersecting')
                slicer.util.warningDisplay('Valid paths failed secondary intersection check, this is less accurate and insertion paths may simply be too close to critical structure. Optimal path will be determined by insertion angle but this should be validated.')
            elif maxDist == 0:
                logging.debug('logic.process failed: no valid paths found between given target and entry points with given constraints')
                slicer.util.errorDisplay('Unexpected Error: no valid paths found, please check parameters and input data.')
                return False
            
            furthestPathIndexes = np.where(pathDistancesToVolumes == maxDist)[0]
            optimalPathIndexes = np.intersect1d(optimalPathIndexes,furthestPathIndexes)

        # Further refine by choosing paths with smallest insertion angles
        validPathAngles = pathProcessor.getValidPathAngles()
        if validPathAngles is not None:
            filteredPathAngles = validPathAngles[optimalPathIndexes]
            minAngle = filteredPathAngles.min()
            lowestAngleIndexes = np.where(filteredPathAngles == minAngle)[0]
            optimalPathIndexes = optimalPathIndexes[lowestAngleIndexes]

        # Get all lines with optimal features
        optimalTrajectories = []
        for i in optimalPathIndexes:
            path = validPaths.GetCell(i)
            points = path.GetPoints()
            trajectory = [points.GetPoint(0),points.GetPoint(1)]
            optimalTrajectories.append(trajectory)

        # Check if there are no valid paths
        if len(optimalTrajectories) < 1:
            logging.debug('logic.process failed: no valid paths found between given target and entry points with given constraints')
            slicer.util.errorDisplay('Unexpected Error: no valid paths found, please check parameters and input data.')
            return False
        
        # Set entry and target points of first optimal trajectory to the output fiducial
        pathFiducial.RemoveAllControlPoints() 
        [optimalEntry,optimalTarget] = optimalTrajectories[0] #first point of path is entry, same for control points in output fiducial
        pathFiducial.AddControlPoint(optimalEntry[0], optimalEntry[1], optimalEntry[2])
        pathFiducial.AddControlPoint(optimalTarget[0], optimalTarget[1], optimalTarget[2])

        # TESTING: Output path for visualisation
        self.visualisePath(optimalEntry,optimalTarget)

        # TESTING: Output total computation time for evaluation
        stopTime = time.time()
        logging.info(f"Processing completed in {stopTime-startTime:.2f} seconds")
        print(f"Processing completed in {stopTime-startTime:.2f} seconds")
        slicer.util.infoDisplay(f"Optimal path chosen, output fiducial '{pathFiducial.GetName()}' contains the entry and target points (as 1st and 2nd control points respectively). ")



class PathPointProcessor():
    """
    Holds methods to process fiducial points and computes valid paths for insertion planning.

    This class provides methods to:
    - Clean fiducial points to ensure they are within inclusion volumes and outside exclusion volumes.
    - Compute valid paths between target and entry points based on specified constraints.
    - Calculate the angle of intersection between the trajectory and the normal of the insertion volume.
    - Check for intersections with critical volumes using various methods.
    - Compute minimum distances from paths to critical volumes.
    """
    def __init__(self, checkInsertionAngle: bool = False, detailedIntersectionCheck: bool = False, ) -> None:
        """
        Initialise the PathPointProcessor, setiing optional check conditions.

        :param checkInsertionAngle: If True, insertion angles will be checked against maximum limits.
        :param detailedIntersectionCheck: If True, detailed intersection checks using vtkOBBTree will be performed.
        """
        self.checkAngle = checkInsertionAngle
        self.obbIntersection = detailedIntersectionCheck
    
    def getValidPathAngles(self):
        """
        Retrieve the valid path angles computed during path processing.

        :return: An array of valid path angles if available, otherwise None. 
        """
        if self.pathAngles:
            if len(self.pathAngles) > 0:
                return np.array(self.pathAngles)
        if self.checkAngle:
            warnings.warn("No valid path angles found", stacklevel=2)
            logging.debug('self.pathAngles not yet initialised: no valid paths have been computed using self.computePaths()')
        return None
 
    def sampleDistanceMap(self, polyData, distanceMap, spacing, transform=None):
        """
        Sample distances from a distance map array along a path defined by polyData.

        :param polyData: vtkPolyData object containing the path to be sampled.
        :param distanceMap: Numpy array representing the distance map.
        :param spacing: The voxel spacing list[3] in the distance map.
        :param transform: Optional vtkTransform to convert path points from RAS to IJK coordinates. If None, no transformation is applied.
        :return: A list of sampled distance values along the path.
        """
        pathPoints = polyData.GetPoints()
        startPointRAS = np.array(pathPoints.GetPoint(0))
        endPointRAS = np.array(pathPoints.GetPoint(1))

        if transform:
            startPointIJK = np.array(transform.TransformPoint(startPointRAS))
            endPointIJK = np.array(transform.TransformPoint(endPointRAS))
        else:
            startPointIJK = startPointRAS
            endPointIJK = endPointRAS

        pathLength = np.linalg.norm(endPointIJK-startPointIJK)
        samples = int(pathLength / np.mean(spacing)) + 2 #minimum of 2 samples

        distances = []
        for i in range(samples):
            step = i/(samples-1)
            pointIJK = (1-step)*startPointIJK + step*endPointIJK

            # Convert point to voxel indices
            voxelIndices = [int(round(coord)) for coord in pointIJK]
            
            # Ensure the indices are within the image bounds
            bounds = [0, distanceMap.shape[0]-1, 0, distanceMap.shape[1]-1, 0, distanceMap.shape[2]-1]
            within_bounds = all([bounds[2*k] <= voxelIndices[k] <= bounds[2*k+1] for k in range(3)])
            
            if within_bounds:
                # Sample the distance map
                voxelValue = distanceMap[voxelIndices[2], voxelIndices[1], voxelIndices[0]]
                distances.append(voxelValue)
            else:
                distances.append(float('inf'))
        
        return distances

    def computeMinDistances(self, polyData, criticalVolumes):
        """
        Compute the minimum distances from each path in the polyData to the surfaces of critical volumes
        using the SignedMaurerDistanceMap filter.

        :param polyData: vtkPolyData object containing the paths to be evaluated.
        :param criticalVolumes: List of  volumes to avoid.
        :return: An array of minimum distances for each path in the polyData to the critical volumes.
        """

        numPaths = polyData.GetNumberOfCells()
        minDistances = np.array([float('inf')] * numPaths)

        for criticalVolume in criticalVolumes:
            # Get distance map using SignedMaurer filter
            sitkImage = sitkUtils.PullVolumeFromSlicer(criticalVolume.GetID())
            spacing = sitkImage.GetSpacing()
            distanceMap = sitk.SignedMaurerDistanceMap(sitkImage, squaredDistance=False, useImageSpacing=True)
            distanceMapArray = sitk.GetArrayFromImage(distanceMap)

            rasToIjkMatrix = vtk.vtkMatrix4x4()
            criticalVolume.GetRASToIJKMatrix(rasToIjkMatrix)
            transform = vtk.vtkTransform()
            transform.SetMatrix(rasToIjkMatrix)
            transform.Update()

            for i in range(numPaths):
                path = polyData.GetCell(i)
                pathDistances = self.sampleDistanceMap(path, distanceMapArray, spacing, transform)
                minDist = min(pathDistances)
                minDistances[i] = min(minDistances[i], minDist)

        return minDistances
    
    def intersectionAngle(self, entryPos, targetPos, fullVolume, isoValue=1.0):
        """
        Calculate the angle between the trajectory from entry to target points
        and the normal of the surface at the intersection point within the full volume.

        :param entryPos: Entry point of the path in RAS coordinates.
        :param targetPos: Target point of the path in RAS coordinates.
        :param fullVolume: The full volume for insertion.
        :param isoValue: IsoValue for surface extraction using marching cubes.
        :return: The angle in degrees between the trajectory and the normal of the surface at the intersection point,
                or None if no intersection is found.
        """
        transformMatrix = vtk.vtkMatrix4x4()
        fullVolume.GetRASToIJKMatrix(transformMatrix)
        transform = vtk.vtkTransform()
        transform.SetMatrix(transformMatrix)

        entryPosTransformed = transform.TransformPoint(entryPos)
        targetPosTransformed = transform.TransformPoint(targetPos)

        lineSource = vtk.vtkLineSource()
        lineSource.SetPoint1(entryPosTransformed)
        lineSource.SetPoint2(targetPosTransformed)
        lineSource.Update()
        linePolyDataPoints = lineSource.GetOutput().GetPoints()

        imageData = fullVolume.GetImageData()
        marchingCubes = vtk.vtkMarchingCubes()
        marchingCubes.SetInputData(imageData)
        marchingCubes.SetValue(0, isoValue)
        marchingCubes.Update()
        meshIJK = marchingCubes.GetOutput()

        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(meshIJK)
        normals.ComputePointNormalsOn()
        normals.Update()

        obbTree = vtk.vtkOBBTree()
        obbTree.SetDataSet(normals.GetOutput())
        obbTree.BuildLocator()

        intersectionPoints = vtk.vtkPoints()
        intersectedCells = vtk.vtkIdList()
        if obbTree.IntersectWithLine(linePolyDataPoints.GetPoint(0),linePolyDataPoints.GetPoint(1), intersectionPoints, intersectedCells):
            # Get the first intersection point
            if intersectionPoints.GetNumberOfPoints() > 0:
                firstIntersectionPoint = intersectionPoints.GetPoint(0)
                closestPointId = normals.GetOutput().FindPoint(firstIntersectionPoint)
                normalAtIntersection = normals.GetOutput().GetPointData().GetNormals().GetTuple(closestPointId)

                # Calculate the angle between the trajectory and the normal
                trajectoryVector = np.array(targetPos) - np.array(entryPos)
                trajectoryVector = trajectoryVector / np.linalg.norm(trajectoryVector)
                normalVector = np.array(normalAtIntersection)
                dotProduct = np.dot(trajectoryVector, normalVector)
                angle = np.arccos(max(dotProduct,-dotProduct))

                # Convert angle to degrees
                angleDeg = np.degrees(angle)
                return angleDeg

        return None

    def checkIntersectionOBB(self, entryPos, targetPos, criticalVolumes, isoValue=1.0):
        """
        Checks if the trajectory between entry and target points intersects with the critical volumes,
        defined by vtkMRMLLabelMapVolumeNodes, using vtkOBBTree. Points are transformed to the IJK
        space of the critical volumes.

        :param entryPos: Entry point of the path in RAS coordinates.
        :param targetPos: Target point of the path in RAS coordinates.
        :param criticalVolumes: List of critical volumes to check for intersections.
        :param isoValue: IsoValue for surface extraction using marching cubes.
        :return: True if there is an intersection, otherwise False.
        """

        for criticalVolume in criticalVolumes:
            # Get the transformation matrix
            transformMatrix = vtk.vtkMatrix4x4()
            criticalVolume.GetRASToIJKMatrix(transformMatrix)
            transform = vtk.vtkTransform()
            transform.SetMatrix(transformMatrix)

            # Transform entry and target points from RAS to IJK
            entryPosIJK = transform.TransformPoint(entryPos)
            targetPosIJK = transform.TransformPoint(targetPos)

            # Use vtkLineSource to connect the transformed points
            lineSource = vtk.vtkLineSource()
            lineSource.SetPoint1(entryPosIJK)
            lineSource.SetPoint2(targetPosIJK)
            lineSource.Update()
            linePolyDataPoints = lineSource.GetOutput().GetPoints()

            # Setup vtkOBBTree for intersection test
            imageData = criticalVolume.GetImageData()
            marchingCubes = vtk.vtkMarchingCubes()
            marchingCubes.SetInputData(imageData)
            marchingCubes.SetValue(0, isoValue)  # Extract the volume surface as a mesh
            marchingCubes.Update()
            mesh = marchingCubes.GetOutput()

            obbTree = vtk.vtkOBBTree()
            obbTree.SetDataSet(mesh)
            obbTree.BuildLocator()

            # Check for intersection
            intersectionPoints = vtk.vtkPoints()
            intersectedCells = vtk.vtkIdList()
            if obbTree.IntersectWithLine(linePolyDataPoints.GetPoint(0),linePolyDataPoints.GetPoint(1), intersectionPoints, intersectedCells):
                return True  # Intersection found
            
        return False  # No intersection found
    
    def checkIntersection(self, entryPos, targetPos, criticalVolumes):
        """
        Checks if the trajectory between entry and target points intersects with the critical volumes,
        defined by a vtkMRMLLabelMapVolumeNode, using vtkProbeFilter. Lines are found between the points
        as vtkLineSources and are transformed to the IJK space of the critical volumes.

        :param entryPos: Entry point of the path in RAS coordinates.
        :param targetPos: Target point of the path in RAS coordinates.
        :param criticalVolumes: List of critical volumes to check for intersections.
        :return: True if there is an intersection, otherwise False.
        """
        # Use vtkLineSource to connect the original points
        lineSource = vtk.vtkLineSource()
        lineSource.SetPoint1(entryPos)
        lineSource.SetPoint2(targetPos)
        lineSource.Update()

        for criticalVolume in criticalVolumes:
            # Get the image data from the volume
            imageData = criticalVolume.GetImageData()

            # Transform the line coordinates into the image data space
            transform = vtk.vtkTransform()
            matrix = vtk.vtkMatrix4x4()
            criticalVolume.GetRASToIJKMatrix(matrix)
            transform.SetMatrix(matrix)
            # transform.Inverse()

            # Transform the line points to IJK space
            transformFilter = vtk.vtkTransformPolyDataFilter()
            transformFilter.SetTransform(transform)
            transformFilter.SetInputConnection(lineSource.GetOutputPort())
            transformFilter.Update()

            # Setup vtkProbeFilter to sample along the line
            probeFilter = vtk.vtkProbeFilter()
            probeFilter.SetInputConnection(transformFilter.GetOutputPort())
            probeFilter.SetSourceData(imageData)
            probeFilter.Update()

            # Check the probed values along the line
            probedData = probeFilter.GetOutput().GetPointData().GetScalars()
            for i in range(probedData.GetNumberOfTuples()):
                if probedData.GetTuple1(i) > 0:  # Assuming non-zero values indicate intersection
                    print("Intersection found")
                    return True  # Intersection found

        return False  # No intersections found with any volumes
    
    def computePaths(self, targetFiducials: vtkMRMLMarkupsFiducialNode, 
                     entryFiducials: vtkMRMLMarkupsFiducialNode, 
                     outputPathsPolyData: vtk.vtkPolyData,
                     criticalVolumes: List[vtkMRMLLabelMapVolumeNode]=None,
                     fullVolume: vtkMRMLLabelMapVolumeNode=None,
                     maximumLength: float=100,
                     maximumAngle: Annotated[float, WithinRange(0, 90)]=55.0):
        """
        Compute valid paths between target and entry fiducials, avoiding critical volumes
        and considering maximum length and angle constraints. Valid paths are
        stored in the provided output vtkPolyData object.

        :param targetFiducials: Contains target points (path ends).
        :param entryFiducials: Contains entry points (path starts).
        :param outputPathsPolyData: vtkPolyData object to store the computed valid paths.
        :param criticalVolumes: List of volumes to avoid.
        :param fullVolume: The full volume for insertion.
        :param maximumLength: Upper limit for accepted trajectory length.
        :param maximumAngle: Maximum accepted angle deviation from the normal of fullVolume.
        :return: None
        """

        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        self.pathAngles = []

        # Iterate through all pairs of entry and target fiducials
        for i in range(targetFiducials.GetNumberOfControlPoints()):
            for j in range(entryFiducials.GetNumberOfControlPoints()):
                targetPos = [0, 0, 0]
                entryPos = [0, 0, 0]
                targetFiducials.GetNthControlPointPosition(i, targetPos)
                entryFiducials.GetNthControlPointPosition(j, entryPos)

                length = np.linalg.norm(np.array(targetPos)-np.array(entryPos))
                # Skip point pair if path length is above max limit
                if length > maximumLength:
                    continue

                # Skip point pair if intersection is found
                if criticalVolumes:
                    if self.obbIntersection:
                        if self.checkIntersectionOBB(entryPos, targetPos, criticalVolumes):
                            continue
                    else:
                        if self.checkIntersection(entryPos, targetPos, criticalVolumes):
                            continue

                # Skip point pair if insertion angle is above max limit
                if fullVolume and self.checkAngle:
                    angle = self.intersectionAngle(entryPos, targetPos, fullVolume)
                    if angle is None:
                        warnings.warn("Paths found outside of full insertion volume and removed, check input data.", stacklevel=2)
                        logging.debug('intersectionAngle() returns None for valid insertion path: suggests incorrect target or full Volumes')
                        continue
                    if angle > maximumAngle:
                        continue
                    else:
                        self.pathAngles.append(angle) #angle must be stored after all other checks to ensure matching index

                # Add points to the points array
                targetIndex = points.InsertNextPoint(targetPos)
                entryIndex = points.InsertNextPoint(entryPos)

                # Create a line between these two points
                line = vtk.vtkLine()
                line.GetPointIds().SetId(0, entryIndex)
                line.GetPointIds().SetId(1, targetIndex)
                lines.InsertNextCell(line)

        # Add the points and lines to the polydata
        outputPathsPolyData.SetPoints(points)
        outputPathsPolyData.SetLines(lines)

        # Testing and Visualise:
        # print(f"Number of Points: {outputPathsPolyData.GetNumberOfPoints()}")
        # print(f"Number of Lines: {outputPathsPolyData.GetNumberOfLines()}")
        # modelNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode")
        # modelNode.SetName('Visualised PolyData')
        # modelNode.SetAndObservePolyData(outputPathsPolyData)
        # print(f"checkIntersection average completion in {np.mean(np.array(self.checkIntersectionTimer)):.4f} seconds")
        # print(f"checkIntersectionOBB average completion in {np.mean(np.array(self.checkIntersectionOBBTimer)):.4f} seconds")

    def cleanPoints(self, inputFiducials: vtkMRMLMarkupsFiducialNode, 
                    outputFiducials: vtkMRMLMarkupsFiducialNode, 
                    inclusionVolume: vtkMRMLLabelMapVolumeNode=None, 
                    exclusionVolumes: List[vtkMRMLLabelMapVolumeNode]=None):
        """
        Clean up fiducial points by removing invalid points based on inclusion and exclusion volumes.

        :param inputFiducials: Input fiducial node containing points to be cleaned.
        :param outputFiducials: Output fiducial node to store valid points.
        :param inclusionVolume: Volume within which points must lie to be considered valid.
        :param exclusionVolumes: List of volumes within which points must not lie.
        """

        # Clean up by removing all points from the output fiducial
        outputFiducials.RemoveAllControlPoints()

        # Transform for the inclusion volume
        inclusionTransform = vtk.vtkTransform()
        if inclusionVolume:
            mat = vtk.vtkMatrix4x4()
            inclusionVolume.GetRASToIJKMatrix(mat)
            inclusionTransform.SetMatrix(mat)

        # Transforms for the exclusion volumes
        exclusionTransforms = []
        if exclusionVolumes:
            for volume in exclusionVolumes:
                mat = vtk.vtkMatrix4x4()
                volume.GetRASToIJKMatrix(mat)
                transform = vtk.vtkTransform()
                transform.SetMatrix(mat)
                exclusionTransforms.append(transform)

        for x in range(inputFiducials.GetNumberOfControlPoints()):
                pos = [0, 0, 0]
                inputFiducials.GetNthControlPointPosition(x, pos)
                validPoint = True

                # Transform position for inclusion volume
                if inclusionVolume:
                    inclusionInd = inclusionTransform.TransformPoint(pos)
                    inclusionPixelValue = inclusionVolume.GetImageData().GetScalarComponentAsDouble(int(inclusionInd[0]), int(inclusionInd[1]), int(inclusionInd[2]), 0)
                    if inclusionPixelValue != 1:
                        continue  # Skip if not within the inclusion volume

                # Check against any/all exclusion volumes using their respective transforms
                if exclusionVolumes:
                    for idx, exclusionVolume in enumerate(exclusionVolumes):
                        exclusionInd = exclusionTransforms[idx].TransformPoint(pos)
                        exclusionPixelValue = exclusionVolume.GetImageData().GetScalarComponentAsDouble(int(exclusionInd[0]), int(exclusionInd[1]), int(exclusionInd[2]), 0)
                        if exclusionPixelValue != 0:
                            validPoint = False
                            break  # Stop checking further if one exclusion volume already excludes this point

                if validPoint:
                    outputFiducials.AddControlPoint(pos[0], pos[1], pos[2])


#
# PathPlanningTest
#


class PathPlanningTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """Do whatever is needed to reset the state - typically a scene clear will be enough."""
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_PathPlanning1()

    def test_PathPlanning1(self):
        """

        """

        self.delayDisplay("Starting the test")

        self.delayDisplay("Test passed")

    def test_PathPlanning1(self):
        """Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        self.delayDisplay("Starting the test")

        self.delayDisplay("Test passed")

