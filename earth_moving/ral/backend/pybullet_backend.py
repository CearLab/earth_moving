import pybullet as p
import pybullet_data
import time as t
import os
import matplotlib.pyplot as plt
import numpy as np
from ral.backend.base_backend import BaseBackend

class PybulletBackend(BaseBackend):
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._gui = simulation.get('gui')  
        if self._gui:
            self._physicsClient = p.connect(p.GUI)
        else:
            self._physicsClient = p.connect(p.DIRECT)
        self._gravity = simulation.get('gravity')
        p.setGravity(self._gravity[0], self._gravity[1], self._gravity[2])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  #  PyBullet_data package (see doc)       
        self._ID = []
        self._ID.append(p.loadURDF("plane.urdf"))
        self._ID.append(p.loadURDF('r2d2.urdf', basePosition=[0.0, 0.0, 1]))        
        
    def step(self):
        p.stepSimulation()
        t.sleep(self._timedelta)