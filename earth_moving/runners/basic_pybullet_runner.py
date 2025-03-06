from scenarios.basic_pybullet_scenario import basic_pybullet_scenario
import numpy as np

pybullet_running_config = {
    'backend_type': 'pybullet',
    'environment':
        {
          'name': 'earth_moving',  
          'aggregates':
              {
                'min_pos': (-1.0, -1.0),
                'max_pos': (+1.0, +1.0),
                'num_clusters': 5,
                'max_per_cluster': 20,
                'max_radius': 0.3,
                'aggregate_urdf': 'clutter/urdf/pebbles/pebbles.urdf'
              }
        },        
    'simulation':
        {
            'timedelta': 1.0/240,
            'gravity': (0, 0, -9.81),
            'gui': True
        },
    'sensor':
        {
         'name': 'rgb_camera',
         'imgW': 320,   # :param imgW: Integer, the width of the rendered image in pixels.
         'imgH': 320,   # :param imgH: Integer, the height of the rendered image in pixels. 
         'camera_target_pose': [1.0, 0.0, 0.0], # :param camera_target_pos: List, the [x, y, z] coordinates of the target position the camera focuses on.
         'camera_distance': 3.0,   # :param camera_distance: Float, the distance of the camera from the target position. 
         'yaw': 0,  # :param yaw: Float, the yaw angle of the camera in degrees.
         'pitch': -90, # :param pitch: Float, the pitch angle of the camera in degrees.
         'roll': 0,   # :param roll: Float, the roll angle of the camera in degrees.
         'up_axis_index': 2, # :param up_axis_index: Integer, the index of the axis that points upwards. Default is 2 (z-axis).        
         'fov': 60.0, # :param fov: Float, the field of view (FOV) of the camera in degrees. 
         'aspect_ratio': 1.0, # :param aspect_ratio: Float, the aspect ratio of the camera view (width/height).
         'near': 0.1,   # :param near: Float, the distance to the near clipping plane. 
         'far': 20,     # :param far: Float, the distance to the far clipping plane.
         'save_path': 'outputs/fig/',         
        }    
    }


if __name__ == '__main__':
    basic_pybullet_scenario(pybullet_running_config)