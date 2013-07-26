from __future__ import print_function

from dynamic_graph import plug
from dynamic_graph.sot.hrp2_14.robot import Robot

from dynamic_graph.sot.motion_planner.feet_follower import SwayMotionCorrection,RobotPositionFromVisp, GoToOnePosition

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
          (-1.,  0., 0., 0.),
          ( 0., -1., 0., 0.),
          ( 0.,  0., 0., 1.))
         )

     plug(self.rosExport.signal(self.objectName),
          self.robotPositionFromVisp.cMo)
     self.robotPositionFromVisp.cMoTimestamp.value=(0.0,0.0)

     self.frameName='cameraBottomLeft'

    
     # Plug wMc/wMr to robotPositionFromVisp
     plug(self.robot.frames[self.frameName].position,
         self.robotPositionFromVisp.wMc)
     plug(self.robot.dynamic.waist, self.robotPositionFromVisp.wMr)

     self.robotPositionFromVisp.plannedObjectPosition.value = \
      (( 1.,  0., 0., 0.),
       ( 0.,  0.,-1., 0.),
       ( 0.,  1., 0., 0.),
       ( 0.,  0., 0., 1.))

     # -- Sway Control Part --
  
     # Plug the visp cMo into Dune Sway Control
     plug(self.rosExport.signal(self.objectName),
         self.smc.cMo)
     plug(robot.dynamic.Jcom,self.smc.Jcom)
     plug(robot.pg.dcomref, self.smc.inputPgVelocity)
     plug(robot.device.control, self.smc.qdot)
     plug(self.robot.frames[self.frameName].position,
         self.smc.wMcamera)
     plug(self.robot.dynamic.waist, self.smc.wMwaist)
    
     # Build the go to one position entity.
     self.go21position = GoToOnePosition('GoToOnePosition')
    
     plug(self.robotPositionFromVisp.position,
         self.go21position.robotPosition);

     self.go21position.targetPosition.value=(0.45,-1.45,2.45);

   def connectgo2pg(self):
     # Plug go to one position to the pattern generator.
     plug(self.go21position.pgVelocity,
          self.robot.pg.velocitydes)


    

  




