from ral.backend.base_backend import BaseBackend
from ral.environment.environment_site_preparation_backend import SitePreparationEnvironmentBackend
from ral.sensor.sensor_backend import BaseSensorBackend

def basic_pybullet_scenario(config):
    
    # init backend
    Basebackend = BaseBackend()    
    Backend = Basebackend.initiate_backend(**config)
    
    # load aggregates
    BaseEnvironment = SitePreparationEnvironmentBackend()
    Environment = BaseEnvironment.initiate_environment(**config)
    Environment.generate_aggregates()        
    Backend.load_aggregates(Environment._aggregates_positions, Environment._aggregate_urdf)
    
    # init camera    
    Camera = Backend.initiate_rgb_sensor(**config) # TODO: is it correct that I don't create a BaseSensorBackend object here?
     
    while True:
        Backend.step()
        _data = Camera.get_data()
        Camera.save_data(data=_data,path=Camera._save_path, name='cam_image')