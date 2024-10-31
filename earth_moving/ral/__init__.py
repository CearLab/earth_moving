def get_backend(**backend_kwargs):
    backend_type = backend_kwargs.get('backend_type')
    timedelta = backend_kwargs.get('timedelta')
    if backend_type == 'pybulllet':
        backend = PybulletBackend(timedelta)
    elif backend_type == 'mujoco':
        backend = ...
    else:
        raise ValueError()
    return backend