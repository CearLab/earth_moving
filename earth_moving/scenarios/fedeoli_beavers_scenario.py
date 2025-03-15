from ral.backend.base_backend import BaseBackend
from ral.environment.environment_beavers_backend import BeaversEnvironmentBackend
from IPython.display import clear_output

def fedeoli_beavers_scenario(config):
    
    # init environment
    BaseEnvironment = BeaversEnvironmentBackend()
    Environment = BaseEnvironment.initiate_environment(**config)
    Environment.generate_vegetation_map()
    
    # init backend
    Basebackend = BaseBackend()     
    Backend = Basebackend.initiate_backend(**config)
    Backend.read_environment(Environment)
        
    # init agents    
    Backend.generate_agents(**config)        
    
    # cycle
    for i in range(10):
    # while True:
        Backend.plot_environment_with_heatmap(Environment)
        Backend.step()
        clear_output(wait=True)
        