from dynamic_graph.sot.motion_planner.feet_follower import SimuObjectInCam, SwayMotionCorrection
from dynamic_graph.sot.motion_planner.sway_motion_correction import *
from dynamic_graph.sot.core.math_small_entities import Inverse_of_matrixHomo

def AddSimu(dunesc):
  SOIC=SimuObjectInCam("SOIC")
  plug(dunesc.rosExport.signal(dunesc.objectName),SOIC.cMo)
  plug(dunesc.robot.frames[dunesc.frameName].position,SOIC.wMc)
  SOIC.initialize(0)
  I = (( 0., -1., 0., 0.),
       ( 0., 0., -1., 0.),
       ( 1., 0., 0., 0. ),
       ( 0., 0., 0., 1. ))
  DgToVisp = Multiply_of_matrixHomo("DgToVisp")
  DgToVisp.sin1.value = I
  plug(SOIC.simu_cMo, DgToVisp.sin2)
  dunesc.rosImport.add('matrixHomoStamped', 'cMoToVisp', '/cMo_simulated')
  plug(DgToVisp.sout, dunesc.rosImport.cMoToVisp)
