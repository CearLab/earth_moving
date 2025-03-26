from ral.backend.base_backend import BaseBackend
from ral.environment.environment_site_preparation_backend import SitePreparationEnvironmentBackend
from ral.sensor.sensor_backend import BaseSensorBackend
from ral.robot.robot_backend import BaseRobotBackend

def ari_scenario(config):
    
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
    
    # init shovel
    BaseRobot = BaseRobotBackend()    
    Shovel = Backend.initiate_robot(**config)
    _shovel_corners = Shovel.define_shovel_area([Shovel._start_pos], [Shovel._start_orientation])    
    
    # shovel visualization
    Shovel.draw_shovel(_shovel_corners)    
    
    # trajectory visualization
    _trajectory_pos, _trajectory_orientation = Shovel.generate_trajectory()    
    _trajectory_corners = Shovel.define_shovel_area(_trajectory_pos, _trajectory_orientation)        
    Shovel.draw_trajectory(_trajectory_corners)
    
    # prediction area visualization
    _prediction_corners = Shovel.define_prediction_area()       
        
    while True:
        Backend.step()
        
        # update the environment        
        positions = Backend.get_aggregates_positions()
        Environment.update_aggregates_positions(positions)
        
        # cluster the aggregates    
        Shovel.cluster_positions(Environment._aggregates_positions)              
        
        # draw stuff
        if abs(Backend._current_time - Backend._update_period) <= 1e-2:
            Backend.color_aggregates_in_clusters(Shovel._cluster_labels)
            Shovel.draw_prediction_area(_prediction_corners)
            Shovel.draw_probabilities(_prediction_corners)
            point_list = Shovel.compute_gaussians()
            Backend.draw_gaussians(point_list)  
        
    #     _data = Camera.get_data()
    #     Camera.save_data(data=_data,path=Camera._save_path, name='cam_image')