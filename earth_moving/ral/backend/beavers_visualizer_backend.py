import time as t
import os
import matplotlib.pyplot as plt
import numpy as np

from ral.backend.base_backend import BaseBackend
from ral.environment.environment_beavers_backend import BeaversEnvironmentBackend
from ral.sensor.sensor_backend import BaseSensorBackend
from ral.robot.robot_backend import BaseRobotBackend

class BeaversVisualizerBackend(BaseBackend):
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._gui = simulation.get('gui')
        
    def step(self, environment: BeaversEnvironmentBackend) -> None:
        environment._current_time += self._timedelta