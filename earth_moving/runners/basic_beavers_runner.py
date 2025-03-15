from scenarios.fedeoli_beavers_scenario import fedeoli_beavers_scenario
import numpy as np

beavers_running_config = {
    'backend_type': 'beavers_visualizer',
    'environment':
        {
          'name': 'beavers_environment',          
          'number_vegetation_clusters': 3,
          'minimum_vegetation': 2,
          'maximum_vegetation': 5,
        },        
    'simulation':
        {
            'timedelta': 1.0,
            'gui': True,
            'number_of_agents': 1,
            'width': 100,
            'height': 50,
        },
    'robot':
        {
            'name': 'beavers_robot',
            'position': [50, 25],
            'vegetation_quality_range': [2, 10],
        },
    }

if __name__ == '__main__':
    fedeoli_beavers_scenario(beavers_running_config)