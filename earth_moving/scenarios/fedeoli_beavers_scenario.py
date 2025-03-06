from ral.backend.base_backend import BaseBackend
from ral.environment.environment_beavers_backend import BeaversEnvironmentBackend

def fedeoli_beavers_scenario(config):
    
    # init backend
    Basebackend = BaseBackend()    
    Backend = Basebackend.initiate_backend(**config)
    
    # create beavers environment
    BaseEnvironment = BeaversEnvironmentBackend()
    Environment = BaseEnvironment.initiate_environment(**config)
    
    # cycle
    while True:
        Backend.step(Environment)