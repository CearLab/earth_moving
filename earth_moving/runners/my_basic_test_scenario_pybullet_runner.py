from earth_moving.scenarios.basic_test_scenario import basic_test_scenario
pybullet_running_camera_config = {
    'backend_type': 'pybullet',    
    'sensor':
        {'imgW': 180,
         'imgH': 360,
         'camera_pose_method': lambda t: 5*t + 2}, # TODO: still need to think about `t`
    'robot':
        {'field1': 'blah',
         'field2': 'blahblah',
    'simulation':
        {
            'timedelta': 1.0/30,
        }
    }
}


if __name__ == '__main__':
    basic_test_scenario(pybullet_running_camera_config)