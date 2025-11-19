import numpy as np

def DN_box(side, step=1):
    """Generate a box of neighbours with given side length and step size."""
    assert side % 2 == 1, "Side length must be odd."
    half_side = side // 2
    neighbours = []
    
    for i in range(-half_side, half_side + 1, step):
        for j in range(-half_side, half_side + 1, step):
            # Always exclude (0,0) - the center point
            if not (i == 0 and j == 0):
                neighbours.append([i, j])
    
    return neighbours