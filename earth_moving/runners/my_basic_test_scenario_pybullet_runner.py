import sys
PATH = '/home/fedeoli/Documents/Work/earth_moving/earth_moving/'
sys.path.append(PATH)

from scenarios.basic_test_scenario import basic_test_scenario

pybullet_running_camera_config = {
    'backend_type': 'pybullet',    
    'sensor':
        {
         'imgW': 400,
         'imgH': 400,
         'camera_pose': [10.0, 10.0, 5.0],
         'camera_up_vector': [0.0, 0.0, 1.0],
         'target_pose': [0.0, 0.0, 0.0],
         'fov': 30,
         'near': 0.0,
         'far': 10,
         'save_path': 'earth_moving/outputs/fig/',
         'camera_pose_method': lambda t: 5*t + 2 # TODO: still need to think about `t`
        },
    'robot':
        {
        },
    'simulation':
        {
            'timedelta': 1.0/240,
            'gravity': (0, 0, -9.81),
            'gui': False
        }
    }


if __name__ == '__main__':
    basic_test_scenario(pybullet_running_camera_config)