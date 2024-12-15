import numpy as np

class RobotMovement:
    def __init__(self, movement_type, length, curvature, Nwaypoints):
        self.movement_type = movement_type
        self.length = length
        self.curvature = curvature
        self.Nwaypoints = Nwaypoints

    def generate_waypoints(self, start_point, start_orientation):
        x, y, z = start_point
        roll, pitch, yaw = start_orientation

        waypoints = []
        delta_theta = self.length / self.curvature if self.curvature != 0 else 0

        if self.movement_type == "forward":
            for i in range(self.Nwaypoints):
                dx = self.length * np.cos(yaw) * (i / (self.Nwaypoints - 1))
                dy = self.length * np.sin(yaw) * (i / (self.Nwaypoints - 1))
                waypoints.append([x + dx, y + dy, z])
            final_yaw = yaw  # Forward movement keeps the same yaw

        elif self.movement_type == "backward":
            yaw = yaw + np.pi
            for i in range(self.Nwaypoints):
                dx = self.length * np.cos(yaw) * (i / (self.Nwaypoints - 1))
                dy = self.length * np.sin(yaw) * (i / (self.Nwaypoints - 1))
                waypoints.append([x + dx, y + dy, z])
            final_yaw = yaw # Backward movement reverses the yaw

        elif self.movement_type == "arc_right":
            cx = x + self.curvature * np.sin(yaw)
            cy = y - self.curvature * np.cos(yaw)
            initial_angle = yaw + np.pi / 2
            for i in range(self.Nwaypoints):
                angle = initial_angle - delta_theta * (i / (self.Nwaypoints - 1))
                nx = cx + self.curvature * np.cos(angle)
                ny = cy + self.curvature * np.sin(angle)
                waypoints.append([nx, ny, z])
            final_yaw = initial_angle - delta_theta - np.pi / 2

        elif self.movement_type == "arc_left":
            cx = x - self.curvature * np.sin(yaw)
            cy = y + self.curvature * np.cos(yaw)
            initial_angle = yaw - np.pi / 2
            for i in range(self.Nwaypoints):
                angle = initial_angle + delta_theta * (i / (self.Nwaypoints - 1))
                nx = cx + self.curvature * np.cos(angle)
                ny = cy + self.curvature * np.sin(angle)
                waypoints.append([nx, ny, z])
            final_yaw = initial_angle + delta_theta + np.pi / 2

        elif self.movement_type == "arc_backward_right":
            # yaw = yaw + np.pi
            cx = x - self.curvature * np.sin(yaw)
            cy = y + self.curvature * np.cos(yaw)
            initial_angle = yaw - np.pi / 2
            for i in range(self.Nwaypoints):
                angle = initial_angle + delta_theta * (i / (self.Nwaypoints - 1))
                nx = cx + self.curvature * np.cos(angle)
                ny = cy + self.curvature * np.sin(angle)
                waypoints.append([nx, ny, z])
            final_yaw = initial_angle + delta_theta + np.pi / 2

        elif self.movement_type == "arc_backward_left":
            # yaw = yaw + np.pi
            cx = x + self.curvature * np.sin(yaw)
            cy = y - self.curvature * np.cos(yaw)
            initial_angle = yaw + np.pi / 2
            for i in range(self.Nwaypoints):
                angle = initial_angle - delta_theta * (i / (self.Nwaypoints - 1))
                nx = cx + self.curvature * np.cos(angle)
                ny = cy + self.curvature * np.sin(angle)
                waypoints.append([nx, ny, z])
            final_yaw = initial_angle - delta_theta - np.pi / 2

        return waypoints, (roll, pitch, final_yaw)
