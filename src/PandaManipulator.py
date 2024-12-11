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
    
    def pick(self)->None:
        self.manipulator.closeGripper(self)
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

    def move(self,x,y,z)->None: # overwrite generic Manipulator.py move
        PandaControl.move_to_camera_coordinates(self,x, y, z)
    
    def moveHome(self)->None:  # overwrite generic Manipulator.py moveHome
        PandaControl.move_to_home(self)
    
    def isMoving(self)->bool:  # overwrite generic Manipulator.py isMoving # but also idk if its necessary
        raise NotImplementedError
