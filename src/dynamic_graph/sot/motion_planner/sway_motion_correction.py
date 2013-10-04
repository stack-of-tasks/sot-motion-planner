from __future__ import print_function

from dynamic_graph import plug
from dynamic_graph.sot.hrp2_14.robot import Robot

from dynamic_graph.sot.motion_planner.feet_follower import SwayMotionCorrection,RobotPositionFromVisp, GoToOnePosition, SimuObjectPositionInCamera
from dynamic_graph.sot.core.math_small_entities import Inverse_of_matrixHomo
from dynamic_graph.ros import *
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
from dynamic_graph.sot.pattern_generator.walking import *

class sway_control:

   def createSimuObject(self):
     # Create object to simulate the object position in the camera.
     self.simuObjectPosInCamera = SimuObjectPositionInCamera("simuObjectPositionInCamera")
     self.simuObjectPosInCamera.wpgMoinit.value=((0.272231025415,   0.009978806549,  -0.962180176589,   0.295006679402), \
                                                 (0.263060237734,  -0.962624644041,   0.064444596419,   0.297909336733), \
                                                 (-0.925575269832,  -0.270655164562,  -0.264681321160,   4.557856952518),\
                                                    (0,0,0,1))
     # Plug the camera position.
     plug(self.robot.frames[self.frameName].position,
          self.simuObjectPosInCamera.wMcam)

     # Plug the com position.
     plug(self.robot.dynamic.com, self.simuObjectPosInCamera.wMcom)

     # Plug the pg in the simulator
     plug(self.robot.pg.comref, self.simuObjectPosInCamera.wpgMcom)
     plug(self.robot.pg.comattitude, self.simuObjectPosInCamera.wpgMcomAtt)     

   def __init__(self, robot):

     self.robot = robot

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

     self.createSimuObject()


     
class decoupled_sway_control(sway_control):

   def __init__(self,robot):
     sway_control.__init__(self,robot)
     # Build the go to one position entity.
     self.go21position = GoToOnePosition('GoToOnePosition')
    
     plug(self.robotPositionFromVisp.position,
         self.go21position.robotPosition);

     self.go21position.setGains((1.0,1.0,0.0))
     self.go21position.setUpperLimit((0.1,0.03,0.0))
     self.go21position.setBottomLimit((-0.1,-0.03,0.0))
     self.go21position.targetPosition.value=(2.8,-1.4,0.0)

   def connectgo2pg(self):
      # Plug go to one position to the pattern generator.
      plug(self.go21position.pgVelocity,
          self.robot.pg.velocitydes)

class garcia_sway_control(sway_control):

   def __init__(self,robot):
      sway_control.__init__(self,robot)

      self.pg=robot.pg

      # Set the intrinsic camera parameters for the cost function in the visual servoed PG.
      # TO DO take it from ros and build the appropriate string

      # Set the landmarks position.
      self.pg.parseCmd(":SetAlgoForZmpTrajectory Garcia")
      self.pg.parseCmd(":singlesupporttime 0.7")
      self.pg.parseCmd(":setNumberOfLandMarks 4")
      self.pg.parseCmd(":setDesiredAngle 0.0")
      self.pg.parseCmd(":setAngleErrorGain 0.05")
      #self.pg.parseCmd(":setLandMarksPositions 4.191465 0.412066 1.834407 4.442912 1.326137 1.773185 3.917841 1.345496 -0.094307 3.666394 0.431425 -0.033085 ")
      #self.pg.parseCmd(":setFinalLandMarks 0.13041 0.13008 0.38875 0.10254 0.46618 -0.52584 0.16374 -0.55278 0.13041 0.13008")
      self.pg.parseCmd(":setLandMarksPositions 4.05690 -0.41207 0.29624 4.31516 -1.32614 0.31287 4.12236 -1.34550 2.24317 3.86410 -0.43142 2.22654")
      self.pg.parseCmd(":setFinalLandMarks 0.13041 0.13008 0.38875 0.10254 0.46618 -0.52584 0.16374 -0.55278 0.13041 0.13008")
      self.pg.setSensorTransformation(((0,-0.17365,0.98481,0),(-1.,0.,0.,0.),(0.,-0.98481,-0.17365,0.),(0.,0.,0.,1.)))
      self.pg.parseCmd(":numberstepsbeforestop 2")
      self.pg.parseCmd(":setfeetconstraint XY 0.02 0.02")
      plug(self.rosExport.signal(self.objectName),
           self.pg.objectpositionincamera)

   def start(self, robot):
      robot.startTracer()
      robot.pg.parseCmd(":VSOnline 1")
   
class dune_sway_control(sway_control):

   def __init__(self,robot):
      sway_control.__init__(self,robot)
      # Create sway motion compensation entity
      self.smc = SwayMotionCorrection('smc')
      # -- Sway Control Part --
      
      # Plug the visp cMo into Dune Sway Control
      plug(self.rosExport.signal(self.objectName),
         self.smc.cMo)
      self.smc.cMoTimestamp.value = (0., 0.)
      plug(robot.pg.dcomref, self.smc.inputdcom)
      plug(self.robot.frames[self.frameName].position,
         self.smc.wMcamera)
      plug(self.robot.dynamic.waist, self.smc.wMwaist)
      
      #Initialize Herdt pg
      robot.pg.parseCmd(":SetAlgoForZmpTrajectory Herdt")
      robot.pg.parseCmd(":doublesupporttime 0.1")
      robot.pg.parseCmd(":singlesupporttime 0.8")
      robot.pg.parseCmd(":numberstepsbeforestop 2")
      robot.pg.parseCmd(":setfeetconstraint XY 0.02 0.01")

   def setvelocitymax(self,i,j,k):
	   self.setMaximumVelocity(i,j,k)
	   
   def initialize(self,I):
      self.smc.initialize(I,0)
      
   def start_dunesc(self,robot):
     plug(self.smc.outputPgVelocity, robot.pg.velocitydes)

def prepare_gsc(robot):
   gsc=garcia_sway_control(robot)
   return gsc

def prepare_dsc(robot):
   robot.dsc=decoupled_sway_control(robot)
   robot.device.after.addSignal(robot.dsc.go21position.name+'.pgVelocity')
   robot.addTrace(robot.dsc.rosExport.name,'objectInCamera')
   robot.addTrace(robot.dsc.go21position.name,'pgVelocity')
  
def prepare_dunesc(robot,I):
	dunesc=dune_sway_control(robot)
	dunesc.initialize(I)
	return dunesc
	
def initializeandlaunchdunesc(robot,I):
   dunesc=dune_sway_control(robot)
   dunesc.initialize(I)
   dunesc.start_dunesc(robot)
   
def dunetest1(robot):
	dunesc=dune_sway_control(robot)
	dunesc.smc.setMaximumVelocity(0.1,0.1,0.05)
	dunesc.smc.setAxis(1,0,0)
	I=((0.36024424324189963, -0.011043073035810341, -0.93279265421046109, -0.034044911691398186), (0.26315928029913049, -0.95811474655506446, 0.11297488936116869, 0.70448444571656255), (-0.89496998743170975, -0.28617159707359219, -0.34224923465923912, 2.7397275131873364), (0.0, 0.0, 0.0, 1.0))
	dunesc.initialize(I)
	return dunesc

