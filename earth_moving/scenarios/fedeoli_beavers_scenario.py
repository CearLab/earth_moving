from ral.backend.base_backend import BaseBackend
from ral.environment.environment_beavers_backend import BeaversEnvironmentBackend
from IPython.display import clear_output

def fedeoli_beavers_scenario(config):        
    
    # init backend
    Basebackend = BaseBackend()     
    Backend = Basebackend.initiate_backend(**config)    
        
    # init agents    
    Backend.generate_agents(**config)        
    
    # cycle
    # for i in range(2):
    while True:
        Backend.plot_environment_with_heatmap()
        Backend.step()
        clear_output(wait=True)
        
    Backend.plot_environment_with_heatmap()
        