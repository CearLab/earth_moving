from ral.backend.base_backend import BaseBackend

def basic_test_scenario(config):
    Basebackend = BaseBackend()
    _simulation = config.get('simulation')
    _backend = config.get('backend_type')
    Backend = Basebackend.initiate_backend(simulation=_simulation, backend_type=_backend)
    
    _sensor = config.get('sensor')
    _path = _sensor.get('save_path')
    Camera = Backend.initiate_rgb_sensor(name='cam',sensor=_sensor)
    
    # while True:
    Backend.step()
    _data = Camera.get_data()
    # Camera.plot_data(data=_data)
    Camera.save_data(data=_data,path=_path, name='cam_image')