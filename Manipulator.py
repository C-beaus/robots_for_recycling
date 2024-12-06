#!/usr/bin/env python

import EndEffector


class Manipulator:
    def __init__(self, end_effector: EndEffector):
        raise NotImplementedError

    def pickMove(self, x,y,z)->None:
        self.move(x,y,z)
        self.end_effector.pick()
    
    def releaseMove(self, x,y,z)->None:
        self.move(x,y,z)
        self.end_effector.release()
    
    def move(x,y,z)->None:
        raise NotImplementedError
    
    def moveHome()->None:
        raise NotImplementedError
    
    def isMoving()->bool:
        raise NotImplementedError










        







        