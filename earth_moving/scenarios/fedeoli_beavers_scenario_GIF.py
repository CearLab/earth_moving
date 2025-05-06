import os
import imageio
from tqdm import tqdm
from earth_moving.ral.backend.base_backend import BaseBackend
from earth_moving.ral.environment.environment_beavers_backend import BeaversEnvironmentBackend
from IPython.display import clear_output

def fedeoli_beavers_scenario_GIF(config):        
    
    # Create a directory to store the images
    output_dir = config.get('simulation').get('output_dir')
    output_dir_frames = os.path.join(output_dir, 'frames')    
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(output_dir_frames, exist_ok=True)
    
    # number of steps
    number_of_steps = int(config.get('simulation').get('number_of_steps')) * 24
    downsampling = config.get('simulation').get('downsampling')
    
    # Init backend
    Basebackend = BaseBackend()     
    Backend = Basebackend.initiate_backend(**config)    
    
    # Init agents    
    Backend.generate_agents(**config)        
    
    # Cycle
    image_files = []    
    for i in tqdm(range(number_of_steps)):
        
        if i % downsampling == 0:
            Backend.plot_environment_with_heatmap()        
            
            # Set figure size
            Backend._fig.set_size_inches(10, 10)  # Adjust the size as needed
            
            # Save the figure with tight bounding box
            image_path = os.path.join(output_dir_frames, f"frame_{i:04d}.png")
            Backend._fig.savefig(image_path, bbox_inches="tight")
            image_files.append(image_path)
            
            # Clear the figure to avoid overlapping plots
            Backend._fig.clf()
        
        Backend.step()
        clear_output(wait=True)
    
    # Create a GIF from the saved images
    file_name = f"DAYS{number_of_steps/24}\
        _NAGENTS{config.get('simulation').get('number_of_agents')}\
        _SIZE{config.get('environment').get('width')}x{config.get('environment').get('height')}\
        _MEAS{config.get('robot').get('measurement_mode')}\
        _EXP{config.get('robot').get('exploration_mode')}\
        _EXPMAP{config.get('robot').get('exploration_map')}\
        _EXPETA{config.get('robot').get('exploration_eta')}\
        _MAXLD{config.get('robot').get('maximum_load')}\
        _CTRL{config.get('robot').get('controller').get('name')}\
        _CTRLMAP{config.get('robot').get('controller').get('map_repulsive')}\
        _STREAM{config.get('environment').get('streams_number')}\
        _SEED{config.get('simulation').get('seed')}.gif"
    gif_path = os.path.join(output_dir, file_name)
    with imageio.get_writer(gif_path, mode="I", duration=0.1) as writer:
        for image_file in tqdm(image_files):
            writer.append_data(imageio.imread(image_file))            
    
    print(f"GIF saved at {gif_path}")    