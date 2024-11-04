from earth_moving.scenarios.basic_test_scenario import basic_test_scenario

ros_config = {
    'backend_type': 'ROS',
    'sensor':
        {'topic_name': 'rgb/image_raw',
             },
        
    'robot':
        {'field1': 'blah',
         'field2': 'blahblah',
    }
}


if __name__ == '__main__':
    basic_test_scenario(ros_config)