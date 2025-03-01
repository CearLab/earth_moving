from ral.backend.base_backend import BaseBackend

def ari_scenario(config):
    Basebackend = BaseBackend()
    _simulation = config.get('simulation')
    _backend = config.get('backend_type')
    Backend = Basebackend.initiate_backend(simulation=_simulation, backend_type=_backend)
    
    while True:
        Backend.step()