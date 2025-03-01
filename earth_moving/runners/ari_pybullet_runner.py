import sys
PATH = '/home/fedeoli/Documents/Work/earth_moving/earth_moving/'
sys.path.append(PATH)

from scenarios.ari_scenario import ari_scenario

pybullet_running_config = {
    'backend_type': 'pybullet',        
    'simulation':
        {
            'timedelta': 1.0/240,
            'gravity': (0, 0, -9.81),
            'gui': True
        }
    }


if __name__ == '__main__':
    ari_scenario(pybullet_running_config)