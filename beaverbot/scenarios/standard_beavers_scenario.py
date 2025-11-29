from beaverbot.ral.backend.base_backend import BaseBackend
from beaverbot.ral.environment.environment_beavers_backend import BeaversEnvironmentBackend
from IPython.display import clear_output
from tqdm import tqdm

def standard_beavers_scenario(config):        
    
    # init backend
    Basebackend = BaseBackend()     
    Backend = Basebackend.initiate_backend(**config)    
    
    # number of steps (in hours)
    number_of_steps = int(config.get('simulation').get('number_of_steps')) * 24    
    downsampling = config.get('simulation').get('downsampling')
    save_path = config.get('simulation').get('save_path')
        
    # init agents    
    Backend.generate_agents(**config)        
    
    # cycle
    for i in range(number_of_steps):     
        if i % downsampling == 0:
            Backend.plot_environment_with_heatmap(plot_agents=False)
            # Backend.plot_simulation_recap()
                
        Backend.step()
        clear_output(wait=True)
    
    # final plots
    Backend.plot_environment_with_heatmap(plot_agents=False) 
    Backend.plot_simulation_recap()
    Backend.save_environment_map(save_path)
        