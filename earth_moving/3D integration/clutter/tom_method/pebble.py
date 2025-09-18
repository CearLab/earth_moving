import numpy as np
from numpy.random import rand

def scatter_pebbles(engine, minpos, maxpos, pebbleNum, pebble_urdf):
    """
    Scatter pebbles within the given range.

    :param engine: The simulation engine instance.
    :param minpos: Minimum position array.
    :param maxpos: Maximum position array.
    :param pebbleNum: Number of pebbles.
    :param pebble_urdf: URDF file path for the pebbles.
    :return: List of pebble positions.
    """
    pebble_positions = []
    for i in range(pebbleNum):
        startPos = minpos + (rand(3) * (maxpos - minpos))
        startPos[-1] = 0.2
        pebble_positions.append(startPos[:2])  # Append only x and y coordinates
        startOrientation = engine.get_quaternion_from_euler([0, 0, 0])
        engine.load_urdf(pebble_urdf, startPos, startOrientation)
    return pebble_positions
