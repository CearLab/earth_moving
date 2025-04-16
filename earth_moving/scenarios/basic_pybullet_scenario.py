from earth_moving. ral import get_backend
from earth_moving.ral.backend.backend_base import BackendBase
from earth_moving.ral.environment.environment_site_preparation import EnvironmentSitePreparation
from earth_moving.ral.sensor.sensor_backend import BaseSensorBackend

def basic_pybullet_scenario(config):
    
    # init backend
    backend = get_backend(**config['simulation'])
    
    # load aggregates
    BaseEnvironment = EnvironmentSitePreparation()
    environment = BaseEnvironment.initiate_environment(**config)
    environment.generate_aggregates()
    
    for i in range(len(environment._aggregates_positions)):
        environment._aggregates_positions[i] = environment._aggregates_positions[i].tolist()
        orientation = [0, 0, 0, 1] 
        pose = environment._aggregates_positions[i] + orientation
        backend.spawn_object(pose, environment._aggregate_urdf)
        
    return
    
    # init camera    
    Camera = Backend.initiate_rgb_sensor(**config) # TODO: is it correct that I don't create a BaseSensorBackend object here?
     
    while True:
        Backend.step()
        _data = Camera.get_data()
        Camera.save_data(data=_data,path=Camera._save_path, name='cam_image')