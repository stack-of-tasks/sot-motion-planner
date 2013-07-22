from __future__ import print_function

from dynamic_graph import plug
from dynamic_graph.sot.hrp2_14.robot import Robot

from dynamic_graph.sot.motion_planner.feet_follower import SwayMotionCorrection,RobotPositionFromVisp

from dynamic_graph.ros import *

class dune_sway_control:

  def __init__(self, robot):

    self.robot = robot

    # Create sway motion compensation entity
    self.smc = SwayMotionCorrection('smc')

    # Create ros member if it does not exist
    self.ros = Ros(robot)

    # Define shorcuts to reduce code verbosity.
    self.rosImport = self.ros.rosImport
    self.rosExport = self.ros.rosExport

    self.objectName = 'objectInCamera'
    self.rosExport.add('matrixHomoStamped',self.objectName,'/object_position')
  
    self.robotPositionFromVisp = RobotPositionFromVisp('robotPositionFromViSP')
  
    # Convert ViSP frame into usual dynamic-graph frame.
    self.robotPositionFromVisp.setSensorTransformation(
        (( 0.,  0., 1., 0.),
         ( 0., -1., 0., 0.),
         (-1.,  0., 0., 0.),
         ( 0.,  0., 0., 1.))
        )

    plug(self.rosExport.signal(self.objectName),
         self.robotPositionFromVisp.cMo)
    self.frameName='cameraBottomLeft'

    
    # Plug wMc/wMr to robotPositionFromVisp
    plug(self.robot.frames[self.frameName].position,
         self.robotPositionFromVisp.wMc)
    plug(self.robot.dynamic.waist, self.robotPositionFromVisp.wMr)



    

  




