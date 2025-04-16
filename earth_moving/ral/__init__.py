from earth_moving import constants as const
from earth_moving.ral.backend.backend_pybullet import PybulletBackend

def get_backend(**backend_kwargs):
    backend_type = backend_kwargs.get('backend_type')
    if backend_type == const.PYBULLET:
        backend = PybulletBackend(**backend_kwargs)
    elif backend_type == const.MUJOCO:
        backend = ...
    else:
        raise ValueError()
    return backend