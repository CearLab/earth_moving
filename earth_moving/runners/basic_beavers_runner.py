from scenarios.fedeoli_beavers_scenario import fedeoli_beavers_scenario
import numpy as np

beavers_running_config = {
    'backend_type': 'beavers_visualizer',
    'environment':
        {
          'name': 'beavers',
          'width': 100,
          'height': 50,
          'sinuosity': 1.5,
          'number_dams': 3,
          'number_vegetation_clusters': 3,
        },        
    'simulation':
        {
            'timedelta': 1.0,
            'gui': True
        }
    }

if __name__ == '__main__':
    fedeoli_beavers_scenario(beavers_running_config)