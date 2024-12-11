#!/usr/bin/env python3

import sys
import os

from EndEffector import EndEffector
from Manipulator import Manipulator
from panda_hw.src.panda_control import PandaControl
class PandaEndEffector(EndEffector):
    def __init__(self, panda_manipulator:PandaControl):
        self.manipulator = panda_manipulator
        self.closed = False
    
    def pick(self, width:float=-1)->None:
        self.manipulator.closeGripper(self, width)
        self.closed = True
    
    def release(self) -> None:
        self.manipulator.openGripper(self)
        self.closed = False
    
    def isClosed(self)->bool:
        return self.closed
    
class PandaManipulator(PandaControl,Manipulator):
    def __init__(self, end_effector_class=PandaEndEffector):
        PandaControl.__init__(self)
        if end_effector_class is PandaEndEffector:
            self.end_effector = PandaEndEffector(self)
        else: self.end_effector = end_effector_class()

    def move(self,x,y,z,theta)->None: # overwrite generic Manipulator.py move
        PandaControl.move_to_camera_coordinates(self,x, y, z, theta)
    
    def moveHome(self)->None:  # overwrite generic Manipulator.py moveHome
        PandaControl.move_to_home(self)
    
    def isMoving(self)->bool:  # overwrite generic Manipulator.py isMoving # but also idk if its necessary
        raise NotImplementedError
    
    def executeGrasp(self, grasps)->None:
        # This is a flat array. needs to be reshaped like grasps.reshape(-1, 6) where each
        # row would then become [x, y, z, angle, witdh, label]
        grasp = grasps.reshape(-1, 6)[0]
        
        self.move(grasp[0], grasp[1], grasp[2], grasp[3])
        self.end_effector.pick(grasp[4])
        
        self.moveHome() #TODO: Go to dropoff based on label
        self.end_effector.release()
