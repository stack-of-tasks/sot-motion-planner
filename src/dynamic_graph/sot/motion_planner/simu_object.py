from dynamic_graph.sot.motion_planner.feet_follower import SimuObjectInCam, SwayMotionCorrection
from dynamic_graph.sot.motion_planner.sway_motion_correction import *
from dynamic_graph.sot.core.math_small_entities import Inverse_of_matrixHomo

def AddSimu(dunesc):
  dunesc.SOIC=SimuObjectInCam("SOIC")
  plug(dunesc.cMoOnRobotFrame.sout,dunesc.SOIC.cMo)
  plug(dunesc.robot.frames[dunesc.frameName].position,dunesc.SOIC.wMc)
  dunesc.SOIC.initialize(1)
  I = (( 0., -1., 0., 0.),
       ( 0., 0., -1., 0.),
       ( 1., 0., 0., 0. ),
       ( 0., 0., 0., 1. ))
  dunesc.SOIC.DgToVisp = Multiply_of_matrixHomo("DgToVisp")
  dunesc.SOIC.DgToVisp.sin1.value = I
  plug(dunesc.SOIC.simu_cMo, dunesc.SOIC.DgToVisp.sin2)
  dunesc.rosImport.add('matrixHomoStamped', 'cMoToVisp', '/object_position_hint')
  plug(dunesc.SOIC.DgToVisp.sout, dunesc.rosImport.cMoToVisp)
