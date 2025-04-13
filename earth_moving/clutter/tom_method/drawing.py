import pybullet as p

def draw_line(engine, from_pos, to_pos, color=[0, 1, 0], width=3):
    from_pos_3d = list(from_pos) + [0] if len(from_pos) == 2 else from_pos
    to_pos_3d = list(to_pos) + [0] if len(to_pos) == 2 else to_pos
    p.addUserDebugLine(
        lineFromXYZ=from_pos_3d,
        lineToXYZ=to_pos_3d,
        lineColorRGB=color,
        lineWidth=width,
        lifeTime=0
    )

def draw_shapely_shape(engine, shape, height=0.1, color=[1, 0, 0]):
    coords = list(shape.exterior.coords)

    for i in range(len(coords) - 1):
        p.addUserDebugLine(
            lineFromXYZ=[coords[i][0], coords[i][1], 0],
            lineToXYZ=[coords[i + 1][0], coords[i + 1][1], 0],
            lineColorRGB=color,
            lifeTime=0
        )

    for i in range(len(coords) - 1):
        p.addUserDebugLine(
            lineFromXYZ=[coords[i][0], coords[i][1], 0],
            lineToXYZ=[coords[i][0], coords[i][1], height],
            lineColorRGB=color,
            lifeTime=0
        )
        p.addUserDebugLine(
            lineFromXYZ=[coords[i][0], coords[i][1], height],
            lineToXYZ=[coords[i + 1][0], coords[i + 1][1], height],
            lineColorRGB=color,
            lifeTime=0
        )

    for i in range(len(coords) - 1):
        p.addUserDebugLine(
            lineFromXYZ=[coords[i][0], coords[i][1], 0],
            lineToXYZ=[coords[i + 1][0], coords[i + 1][1], 0],
            lineColorRGB=color,
            lifeTime=0
        )
        p.addUserDebugLine(
            lineFromXYZ=[coords[i][0], coords[i][1], height],
            lineToXYZ=[coords[i + 1][0], coords[i + 1][1], height],
            lineColorRGB=color,
            lifeTime=0
        )

def draw_smooth_curve_from_points(engine, points, color=[1, 0, 1]):
    num_points = len(points)
    for i in range(num_points - 1):
        from_pos = list(points[i]) + [0] if len(points[i]) == 2 else points[i]
        to_pos = list(points[i + 1]) + [0] if len(points[i + 1]) == 2 else points[i + 1]
        p.addUserDebugLine(from_pos, to_pos, color, lineWidth=3)
