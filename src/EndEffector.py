#!/usr/bin/env python

class EndEffector:
    def __init__(self):
        self.closed = None
    
    def pick(self)->None:
        self.closed = True
        ## also do the mechanical stuff
    
    def release(self) -> None:
        self.closed = False
        ## also do the mechanical stuff
    
    def isClosed(self)->bool:
        return self.closed