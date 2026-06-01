import numpy as np
import matplotlib.pyplot as plt
import cv2
import copy


class PreprocessEnvironment:
    """
    Original PushEnvironment with MCTS compatibility methods added.
    This maintains your original boundary-based physics and logic.
    """
    def __init__(self, boundary_points, push_width, stochasticity=False, grid_size=20, binary_grid=True):
        self.boundary_points = boundary_points
        self.push_width = push_width
        # self.BB_SIZE = 500                                 # Size of the bounding box in real-world units mm
        self.BB_SIZE = 0.5                                   # Size of the bounding box in real-world units meters
        self.grid_size = grid_size
        self.grid_res = 1.0 / grid_size
        self.grid_real_res = self.BB_SIZE * self.grid_res  # Real-world size of each grid cell
        self.grid_offset = np.array([0.0, 0.0])            # Assuming grid starts at (0,0) in world coordinates
        self.stochastic_push = stochasticity
        self.total_pixels_in_cell = 0.0

        if self.stochastic_push or binary_grid is False:
            print("Stochastic push enabled.")
            self.binary_grid = False
        else:
            print("Deterministic push enabled.")
            self.binary_grid = True

        # self.occupancy_grid, self.total_occupied = self.particles_to_grid(particles, grid_size)

    def binary_img_to_grid(self, binary_img):
        total_occupied = 0

        binary_img_size = binary_img.shape[0]
            
        # Calculate how many fine-grid pixels fit into one sim-grid cell
        ratio = binary_img_size // self.grid_size
        
        if ratio == 0:
            print(f"Error: BINARY_IMG_SIZE ({binary_img_size}) must be larger than ENV_GRID_SIZE ({self.grid_size}).")
            return grid_mask
            
        self.total_pixels_in_cell = float(ratio * ratio)

        if self.total_pixels_in_cell == 0:
            return grid_mask

        if self.binary_grid:
            grid_mask = np.zeros((self.grid_size, self.grid_size), dtype=bool)
            for r in range(self.grid_size):
                for c in range(self.grid_size):
                    r_start = r * ratio
                    r_end = (r + 1) * ratio
                    c_start = c * ratio
                    c_end = (c + 1) * ratio
                    
                    # Slice the block from the binary grid
                    block = binary_img[r_start:r_end, c_start:c_end]

                    # Calculate density
                    filled_pixels = np.sum(block)
                    density = (filled_pixels / (self.total_pixels_in_cell))
                    
                    if density > 0.01:
                        grid_mask[c, r] = True
                        total_occupied += 1
        else:
            grid_mask = np.zeros((self.grid_size, self.grid_size), dtype=float)
            
            for r in range(self.grid_size):
                for c in range(self.grid_size):
                    r_start = r * ratio
                    r_end = (r + 1) * ratio
                    c_start = c * ratio
                    c_end = (c + 1) * ratio
                    
                    # Slice the block from the binary grid
                    block = binary_img[r_start:r_end, c_start:c_end]
                    
                    # Calculate density
                    filled_pixels = np.sum(block)
                    density = (filled_pixels / (self.total_pixels_in_cell))
                    
                    grid_mask[c, r] = density
                    
            print("Downsampling complete.")
            total_occupied = np.sum(grid_mask)
        
        return grid_mask, total_occupied

    def particles_to_grid(self, particles):
        total_occupied = 0
        self.total_pixels_in_cell = 0.0

        if self.binary_grid:
            grid_mask = np.zeros((self.grid_size, self.grid_size), dtype=bool)
            for p in particles:
                idx = self._world_to_grid(p)
                if idx is not None and not grid_mask[idx]:
                    grid_mask[idx] = True
                    total_occupied += 1
        else:
            grid_mask = np.zeros((self.grid_size, self.grid_size), dtype=float)
            binary_img = self.particles_to_binary_img(particles, binary_grid_size=1000, bounding_box=((0,0), (1,1)), particle_radius_world=0.003)
            binary_img_size = binary_img.shape[0]
    
            # Calculate how many fine-grid pixels fit into one sim-grid cell
            ratio = binary_img_size // self.grid_size
            
            if ratio == 0:
                print(f"Error: BINARY_IMG_SIZE ({binary_img_size}) must be larger than ENV_GRID_SIZE ({self.grid_size}).")
                return grid_mask
                
            self.total_pixels_in_cell = float(ratio * ratio)

            if self.total_pixels_in_cell == 0:
                return grid_mask
            
            for r in range(self.grid_size):
                for c in range(self.grid_size):
                    r_start = r * ratio
                    r_end = (r + 1) * ratio
                    c_start = c * ratio
                    c_end = (c + 1) * ratio
                    
                    # Slice the block from the binary grid
                    block = binary_img[r_start:r_end, c_start:c_end]
                    
                    # Calculate density
                    filled_pixels = np.sum(block)
                    density = (filled_pixels / (self.total_pixels_in_cell))
                    
                    grid_mask[c, r] = density
                    
            print("Downsampling complete.")
            total_occupied = np.sum(grid_mask)
        
        return grid_mask, total_occupied
    
    def grid_to_particles(self, grid_mask):
        particles = []
        for i in range(grid_mask.shape[0]):
            for j in range(grid_mask.shape[1]):
                if grid_mask[i, j]:
                    x = (i + 0.5) * self.grid_res
                    y = (j + 0.5) * self.grid_res
                    particles.append((x, y))
        return np.array(particles)
    # def grid_to_particles(self, grid_mask):
    #     particles = []
    #     for i in range(grid_mask.shape[0]):
    #         for j in range(grid_mask.shape[1]):
    #             if grid_mask[i, j]:
    #                 num_in_cell = int(grid_mask[i, j] * self.num_particles)
    #                 x_min, x_max = i * self.grid_res, (i + 1.0) * self.grid_res
    #                 y_min, y_max = j * self.grid_res, (j + 1.0) * self.grid_res
    #                 x_coords = np.random.uniform(x_min, x_max, size=num_in_cell)
    #                 y_coords = np.random.uniform(y_min, y_max, size=num_in_cell)
    #                 particles.extend(zip(x_coords, y_coords))
    #     return np.array(particles)

    def particles_to_binary_img(self,particles, binary_grid_size, bounding_box, particle_radius_world=0.003):
        """
        Converts particles to a fine binary grid.
        """
        print(f"Rasterizing {len(particles)} particles (with size) onto a {binary_grid_size}x{binary_grid_size} binary grid...")
        binary_grid = np.zeros((binary_grid_size, binary_grid_size), dtype=np.uint8)
        
        bb_low = bounding_box[0]
        bb_dims = bounding_box[1] - bounding_box[0]

        # Scale particles from [0, 1] range to [0, binary_grid_size]
        # We use (binary_grid_size - 1e-9) to keep particles from landing exactly on the max edge
        scaled_particles = (particles - bb_low) / bb_dims * (binary_grid_size - 1e-9)
        
        # Convert world radius to pixel radius
        # We scale the radius by the grid size relative to the world width
        if bb_dims[0] == 0: bb_dims[0] = 1.0 # Avoid divide by zero
        particle_radius_pixels = int(np.ceil(particle_radius_world * binary_grid_size / bb_dims[0]))
        print(f"Particle world radius {particle_radius_world} corresponds to a pixel 'radius' of {particle_radius_pixels}")
        
        for p in scaled_particles:
            # p[0] (x) maps to j (col)
            # p[1] (y) maps to i (row)
            j, i = np.floor(p).astype(int)
            
            row_idx = i
            col_idx = j
            
            # Calculate the bounding box for this particle's "square"
            # We use the 'radius' to go from (center - r) to (center + r)
            r_min = max(0, row_idx - particle_radius_pixels)
            r_max = min(binary_grid_size, row_idx + particle_radius_pixels + 1)
            c_min = max(0, col_idx - particle_radius_pixels)
            c_max = min(binary_grid_size, col_idx + particle_radius_pixels + 1)
            
            # "Draw" the filled square onto the binary grid
            # Overlapping particles will just re-write 1s, which is fine.
            if 0 <= row_idx < binary_grid_size and 0 <= col_idx < binary_grid_size:
                binary_grid[r_min:r_max, c_min:c_max] = 1
                
        print("Rasterization complete.")
        return binary_grid

    def hsv_binary_mask(self, rect_env_img):
        hsv = cv2.cvtColor(rect_env_img, cv2.COLOR_BGR2HSV)

        # 3. Define the range for the dark particles
        # These values target dark/brownish colors. 
        # You may need to tune these based on lighting.
        # lower_val = np.array([0, 0, 0])      # Lower bound (H, S, V)
        # upper_val = np.array([180, 255, 100]) # Upper bound (Targets low brightness)
        lower_val = np.array([0, 0, 0])      # Lower bound (H, S, V)
        upper_val = np.array([180, 255, 130]) # Upper bound (Targets low brightness)

        # 4. Create the binary mask
        mask = cv2.inRange(hsv, lower_val, upper_val)

        binary_mask = mask / 255.0

        return binary_mask

    def binary_mask_pybullet(self, rect_env_img):
        _, mask = cv2.threshold(rect_env_img, 40, 255, cv2.THRESH_BINARY_INV)

        binary_mask = mask / 255.0

        return binary_mask

    def transform_raw_image_to_binary_mask_old(self, raw_image, transformation_matrix, rect_size=(700, 500), square_size=500, debug=False):
        # apply perspective transformation to get top-down view
        rect_size = (rect_size[1], rect_size[0])
        warped = cv2.warpPerspective(raw_image, transformation_matrix, rect_size)
        oriented_top_down_img = cv2.flip(cv2.transpose(warped), 0)
        if debug:
            plt.figure()
            plt.imshow(cv2.cvtColor(oriented_top_down_img, cv2.COLOR_BGR2RGB))
        # crop the top-down image to get the square area of interest
        if square_size > min(rect_size):
            square_size = min(rect_size)
        rect_env_img = oriented_top_down_img[:square_size, :square_size]
        rect_env_img = np.asarray(rect_env_img)
        if debug:
            plt.figure()
            plt.imshow(cv2.cvtColor(rect_env_img, cv2.COLOR_BGR2RGB))
        # generate binary mask using HSV color space
        binary_mask = self.hsv_binary_mask(rect_env_img)
        if debug:
            print(binary_mask.shape)
            print(np.unique(binary_mask))
            plt.figure()
            plt.imshow(binary_mask, cmap='gray')

        return binary_mask

    def transform_pybullet_image_to_binary_mask(self, img, rect_size=(1000, 1000), square_size=500, debug=False):
        # crop the top-down image to get the square area of interest
        if square_size > min(rect_size):
            square_size = min(rect_size)
        center = square_size // 2
        start_x = rect_size[1] // 2 - center
        start_y = rect_size[0] // 2 - center
        end_x = start_x + square_size
        end_y = start_y + square_size
        rect_env_img = img[start_y:end_y, start_x:end_x, :]
        rect_env_img = np.asarray(rect_env_img)
        if debug:
            plt.figure()
            plt.imshow(cv2.cvtColor(rect_env_img, cv2.COLOR_BGR2RGB))
        # generate binary mask using HSV color space
        binary_mask = self.binary_mask_pybullet(rect_env_img)
        if debug:
            print(binary_mask.shape)
            print(np.unique(binary_mask))
            plt.figure()
            plt.imshow(binary_mask, cmap='gray')

        return binary_mask

    def transform_raw_image_to_binary_mask_aruco(self, raw_image, transformation_matrix, rect_size=(700, 500), square_size=500, debug=False):
        # apply perspective transformation to get top-down view
        warped = cv2.warpPerspective(raw_image, transformation_matrix, rect_size)
        if debug:
            plt.figure()
            plt.imshow(cv2.cvtColor(warped, cv2.COLOR_BGR2RGB))
        # crop the top-down image to get the square area of interest
        if square_size > min(rect_size):
            square_size = min(rect_size)
        rect_env_img = warped[:square_size, :square_size]
        rect_env_img = np.asarray(rect_env_img)
        if debug:
            plt.figure()
            plt.imshow(cv2.cvtColor(rect_env_img, cv2.COLOR_BGR2RGB))
        # generate binary mask using HSV color space
        binary_mask = self.hsv_binary_mask(rect_env_img)
        if debug:
            print(binary_mask.shape)
            print(np.unique(binary_mask))
            plt.figure()
            plt.imshow(binary_mask, cmap='gray')

        return binary_mask

    def transform_pybullet_image_to_grid(self, img, rect_size=(1000, 1000), square_size=500, debug=False):
        binary_mask = self.transform_pybullet_image_to_binary_mask(img, rect_size, square_size, debug)
        flipped_env_image = cv2.flip(binary_mask, 0)
        grid_mask, total_occupied = self.binary_img_to_grid(flipped_env_image)
        return grid_mask, total_occupied

    def transform_raw_image_to_grid_old(self, raw_image, transformation_matrix, rect_size=(700, 500), square_size=500, debug=False):
        binary_mask = self.transform_raw_image_to_binary_mask_old(raw_image, transformation_matrix, rect_size, square_size, debug)
        flipped_env_image = cv2.flip(binary_mask, 0)
        grid_mask, total_occupied = self.binary_img_to_grid(flipped_env_image)
        return grid_mask, total_occupied
    
    def transform_raw_image_to_grid_aruco(self, raw_image, transformation_matrix, rect_size=(700, 500), square_size=500, debug=False):
        binary_mask = self.transform_raw_image_to_binary_mask_aruco(raw_image, transformation_matrix, rect_size, square_size, debug)
        flipped_env_image = cv2.flip(binary_mask, 0)
        grid_mask, total_occupied = self.binary_img_to_grid(flipped_env_image)
        return grid_mask, total_occupied

    def _world_to_grid(self, pos, grid_size=None):
        if grid_size is None:
            grid_size = self.grid_size
        x, y = pos
        i = int(x * grid_size)
        j = int(y * grid_size)
        if 0 <= i < grid_size and 0 <= j < grid_size:
            return i, j
        return None
    
    def _grid_to_world(self, grid_pos):
        i, j = grid_pos
        x = (i + 0.5) * self.grid_res
        y = (j + 0.5) * self.grid_res
        return x, y
          
    def transform_theta_to_absolute(self, relative_theta, start_point):
        """
        Transform relative theta to absolute theta based on the current state.
        The absolute theta is the angle from the current boundary point to the next.
        """
        if relative_theta is None:
            return None
        # Normalize relative theta to be within [0, pi)
        relative_theta = relative_theta % np.pi

        # Convert relative theta to absolute theta
        if start_point[0] == 0:
            boundary_angle = 3*np.pi / 2
        elif start_point[1] == 1:
            boundary_angle = np.pi
        elif start_point[0] == 1:
            boundary_angle = np.pi / 2
        else:
            boundary_angle = 0

        absolute_theta = boundary_angle + relative_theta
        # Normalize absolute theta to be within [0, 2*pi)
        absolute_theta = absolute_theta % (2 * np.pi)
        # Ensure absolute theta is within [0, 2*pi)
        if absolute_theta < 0:
            absolute_theta += 2 * np.pi
        if absolute_theta > 2 * np.pi:
            absolute_theta -= 2 * np.pi

        return absolute_theta
    
    def _compute_ray_box_intersection(self, origin, direction):
        """
        Compute the intersection length (t) for a ray from origin in given direction
        with the axis-aligned bounding box [0,1]^2.
        """
        tmin = float('-inf')
        tmax = float('inf')

        for i in range(2):  # x and y
            if abs(direction[i]) < 1e-8:
                if origin[i] < 0 or origin[i] > 1:
                    return 0.0  # No intersection
            else:
                t1 = (0.0 - origin[i]) / direction[i]
                t2 = (1.0 - origin[i]) / direction[i]
                t_near = min(t1, t2)
                t_far = max(t1, t2)
                tmin = max(tmin, t_near)
                tmax = min(tmax, t_far)

        if tmax >= tmin and tmax > 0:
            return tmax
        else:
            return 0.0

    def transform_s_to_normed_world(self, s_idx):
        # Get the boundary point [x,y] corresponding to s_idx in normalized world coordinates [0,1]^2
        boundary_point = np.array(self.boundary_points[s_idx])
        return boundary_point
    

    def _rotate_vector(self, vector, angle):
        """
        Rotate a 2D vector by a given angle.
        """
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        rotation_matrix = np.array([[cos_theta, -sin_theta],
                                     [sin_theta, cos_theta]])
        return np.dot(rotation_matrix, vector)

    def draw_environment(self, perticles, pushes=[], current_push=None):
        plt.figure(figsize=(8,8))
        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)

        # Draw boundary
        boundary_x, boundary_y = zip(*self.boundary_points)
        plt.plot(boundary_x + (boundary_x[0],), boundary_y + (boundary_y[0],), 'k-')

        if len(perticles) > 0:
            plt.scatter(perticles[:,0], perticles[:,1], s=10)

        # Draw all pushes
        for push in pushes:
            # start_point, direction = push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'b-')
            plt.fill(rect[:,0], rect[:,1], 'cyan', alpha=0.2)

        # Draw current push in different color
        if current_push is not None:
            # start_point, direction = current_push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * self.push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * self.push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'r-')
            plt.fill(rect[:,0], rect[:,1], 'orange', alpha=0.4)

        plt.title("Aggregates Environment")
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.grid(True)
        plt.show()

    def draw_real_environment(self, perticles, pushes=[], current_push=None):
        plt.figure(figsize=(8,8))
        plt.xlim(0.0, self.BB_SIZE)
        plt.ylim(0.0, self.BB_SIZE)
        real_particles = perticles * self.BB_SIZE
        real_push_width = self.push_width * self.BB_SIZE

        # Draw boundary
        boundary_x, boundary_y = zip(*self.boundary_points)
        plt.plot(boundary_x + (boundary_x[0],), boundary_y + (boundary_y[0],), 'k-')

        if len(perticles) > 0:
            plt.scatter(real_particles[:,0], real_particles[:,1], s=10)

        # Draw all pushes
        for push in pushes:
            # start_point, direction = push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'b-')
            plt.fill(rect[:,0], rect[:,1], 'cyan', alpha=0.2)

        # Draw current push in different color
        if current_push is not None:
            # start_point, direction = current_push
            start_point, end_point, theta = push
            # theta = np.arctan2(direction[1], direction[0])
            # normal = np.array([-np.sin(theta), np.cos(theta)])
            rect_start = np.array(start_point)
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            # rect_end = rect_start + direction * t_max
            rect_end = np.array(end_point)
            rect = np.array([rect_start,
                             rect_start + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([-np.sin(theta), np.cos(theta)]) * real_push_width/2,
                             rect_end + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2,
                             rect_start + np.array([np.sin(theta), -np.cos(theta)]) * real_push_width/2])
            plt.plot(rect[:,0], rect[:,1], 'r-')
            plt.fill(rect[:,0], rect[:,1], 'orange', alpha=0.4)

        plt.title("Aggregates Environment")
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.grid(True)
        plt.show()

    def animate_grid_push(self, action):

        s_idx = self.current_state['s_idx']

        if isinstance(action, (int, np.integer)):
            delta_s, theta = self.action_space[action]
        else:
            delta_s, theta = action
            
        new_state, _, _, _ = self.apply_push(self.current_state, delta_s, theta, print_push=True)

        grid_mask = new_state['grid_mask']

        plt.figure(figsize=(8, 8))
        plt.xlim(-0.05, 1.05)
        plt.ylim(-0.05, 1.05)

        # Draw grid as image
        plt.imshow(grid_mask.T, origin='lower', cmap='Greys',vmin=0, vmax=1, extent=[0, 1, 0, 1], alpha=0.7)

        for i in range(self.grid_size):    # Loop over plot rows (y-axis)
            for j in range(self.grid_size):  # Loop over plot columns (x-axis)

                # Get the value. Because you plotted grid_mask.T,
                # plot cell (row i, col j) corresponds to grid_mask[j, i]
                value = grid_mask[j, i]

                if value > 0.0:
                    # Calculate the center coordinate of the cell
                    x_center = (j + 0.5) * self.grid_res
                    y_center = (i + 0.5) * self.grid_res
                    
                    # Format the text (e.g., to 2 decimal places)
                    text_label = f"{value:.2f}"
                    
                    text_color = "black"

                    plt.text(x_center, y_center, text_label, 
                            ha="center", va="center", color=text_color, fontsize=6)

        # Draw boundary
        bx, by = zip(*self.boundary_points)
        plt.plot(bx + (bx[0],), by + (by[0],), 'k-')

        plt.title("Occupancy Grid")
        plt.grid(True)
        # Add grid lines at cell boundaries
        for i in range(self.grid_size + 1):
            x = i * self.grid_res
            plt.axvline(x=x, color='lightgray', linewidth=0.5, alpha=0.7)
            plt.axhline(y=x, color='lightgray', linewidth=0.5, alpha=0.7)
        plt.show()

    def animate_transition_push(self, action):

        s_idx = self.current_state['s_idx']

        if isinstance(action, (int, np.integer)):
            delta_s, theta = self.action_space[action]
        else:
            delta_s, theta = action
            
        new_state, _, _, _ = self.apply_push(self.current_state, delta_s, theta, print_push=2)

        grid_mask = new_state['grid_mask']

        plt.figure(figsize=(8, 8))
        plt.xlim(-0.05, 1.05)
        plt.ylim(-0.05, 1.05)

        # Draw grid as image
        plt.imshow(grid_mask.T, origin='lower', cmap='Greys',vmin=0, vmax=1, extent=[0, 1, 0, 1], alpha=0.7)

        for i in range(self.grid_size):    # Loop over plot rows (y-axis)
            for j in range(self.grid_size):  # Loop over plot columns (x-axis)

                # Get the value. Because you plotted grid_mask.T,
                # plot cell (row i, col j) corresponds to grid_mask[j, i]
                value = grid_mask[j, i]

                if value > 0.0:
                    # Calculate the center coordinate of the cell
                    x_center = (j + 0.5) * self.grid_res
                    y_center = (i + 0.5) * self.grid_res
                    
                    # Format the text (e.g., to 2 decimal places)
                    text_label = f"{value:.2f}"
                    
                    text_color = "black"

                    plt.text(x_center, y_center, text_label, 
                            ha="center", va="center", color=text_color, fontsize=6)

        # Draw boundary
        bx, by = zip(*self.boundary_points)
        plt.plot(bx + (bx[0],), by + (by[0],), 'k-')

        plt.title("Occupancy Grid")
        plt.grid(True)
        # Add grid lines at cell boundaries
        for i in range(self.grid_size + 1):
            x = i * self.grid_res
            plt.axvline(x=x, color='lightgray', linewidth=0.5, alpha=0.7)
            plt.axhline(y=x, color='lightgray', linewidth=0.5, alpha=0.7)
        plt.show()

        return new_state


    def draw_grid_environment(self, grid_mask, pushes=[], current_push=None, save_path=None):
        plt.figure(figsize=(8, 8))
        plt.xlim(-0.05, 1.05)
        plt.ylim(-0.05, 1.05)

        # Draw grid as image
        plt.imshow(grid_mask.T, origin='lower', cmap='Greys',vmin=0, vmax=1, extent=[0, 1, 0, 1], alpha=0.7)

        for i in range(self.grid_size):    # Loop over plot rows (y-axis)
            for j in range(self.grid_size):  # Loop over plot columns (x-axis)

                # Get the value. Because you plotted grid_mask.T,
                # plot cell (row i, col j) corresponds to grid_mask[j, i]
                value = grid_mask[j, i]

                # if value > 0.0:
                if value:
                    # Calculate the center coordinate of the cell
                    x_center = (j + 0.5) * self.grid_res
                    y_center = (i + 0.5) * self.grid_res
                    
                    # Format the text (e.g., to 2 decimal places)
                    text_label = f"{value:.2f}"
                    
                    text_color = "black"

                    plt.text(x_center, y_center, text_label, 
                            ha="center", va="center", color=text_color, fontsize=6)

        # Draw boundary
        bx, by = zip(*self.boundary_points)
        plt.plot(bx + (bx[0],), by + (by[0],), 'k-')

        # Draw previous pushes
        for start_point, direction in pushes:
            self._draw_push(start_point, direction, color='cyan', alpha=0.2)
        # # Draw previous pushes
        # for s_idx, delta_s, theta in pushes:
        #     self._draw_push(s_idx, delta_s, theta, color='cyan', alpha=0.2)

        # Draw current push
        if current_push is not None:
            start_point, direction = current_push
            self._draw_push(start_point, direction, color='orange', alpha=0.1)

        plt.title("Occupancy Grid")
        # plt.grid(True)
        # Add grid lines at cell boundaries
        for i in range(self.grid_size + 1):
            x = i * self.grid_res
            plt.axvline(x=x, color='lightgray', linewidth=0.5, alpha=0.7)
            plt.axhline(y=x, color='lightgray', linewidth=0.5, alpha=0.7)
        
        if save_path is not None:
            plt.savefig(save_path, bbox_inches='tight')
            plt.close()
        else:
            plt.show()

    # def _draw_push(self, s_idx, delta_s, theta, color='cyan', alpha=0.3):
    def _draw_push(self, start_point, direction, color='cyan', alpha=0.3):
        theta = np.arctan2(direction[1], direction[0])
        normal = np.array([-np.sin(theta), np.cos(theta)])

        rect_start = start_point
        t_max = self._compute_ray_box_intersection(start_point, direction)
        rect_end = rect_start + direction * t_max
        
        if self.shovel_type == 'flat':
            push_normal = normal
        elif self.shovel_type == 'angled_right_15':
            angle_offset = np.pi / 12  # 15 degrees to the right
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'angled_right_20':
            angle_offset = np.pi / 9  # 20 degrees to the right
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'angled_right_25':
            angle_offset = (5 * np.pi) / 36  # 25 degrees to the right
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'angled_left_25':
            angle_offset = -(5 * np.pi) / 36  # 25 degrees to the left
            rot_matrix = np.array([[np.cos(angle_offset), -np.sin(angle_offset)],
                                   [np.sin(angle_offset),  np.cos(angle_offset)]])
            push_dir = rot_matrix @ direction
            push_normal = np.array([-push_dir[1], push_dir[0]])
        elif self.shovel_type == 'concave':
            angle_offset_right = np.pi / 12  # 15 degrees to the right
            angle_offset_left = -np.pi / 12   # 15 degrees to the left
            rot_matrix_right = np.array([[np.cos(angle_offset_right), -np.sin(angle_offset_right)],
                                   [np.sin(angle_offset_right),  np.cos(angle_offset_right)]])
            rot_matrix_left = np.array([[np.cos(angle_offset_left), -np.sin(angle_offset_left)],
                                   [np.sin(angle_offset_left),  np.cos(angle_offset_left)]])
            push_dir_right = rot_matrix_right @ direction
            push_dir_left = rot_matrix_left @ direction
            push_normal_right = np.array([-push_dir_right[1], push_dir_right[0]])
            push_normal_left = np.array([-push_dir_left[1], push_dir_left[0]])
            # For concave, we can average the normals for visualization
            push_normal = (push_normal_right + push_normal_left) / 2

        
        shovel = np.array([
            rect_start + push_normal * (self.push_width / 2),
            rect_start - push_normal * (self.push_width / 2)
        ])

        rect = np.array([
            rect_start + normal * (self.push_width / 2),
            rect_end + normal * (self.push_width / 2),
            rect_end - normal * (self.push_width / 2),
            rect_start - normal * (self.push_width / 2),
            rect_start + normal * (self.push_width / 2)
        ])

        plt.plot(rect[:, 0], rect[:, 1], color=color)
        plt.fill(rect[:, 0], rect[:, 1], color=color, alpha=alpha)
        plt.scatter(*start_point, color='red', s=30)  # Push origin
        plt.plot(shovel[:, 0], shovel[:, 1], color='brown', linewidth=4, label="Shovel Edge")

    def plot_pushes_original_space(self, pushes, boundary_points, rock_positions, shovel_width, title="Push Rectangles"):

        plt.figure(figsize=(8, 8))

        # Plot the particles
        plt.scatter(rock_positions[:, 0], rock_positions[:, 1], s=10, color='black', label="Particles")

        # # Plot boundary
        # bx, by = zip(*boundary_points)
        # plt.plot(bx + (bx[0],), by + (by[0],), 'gray', linestyle='--', label="Boundary")

        # Rectangle colors
        colors_rect = ['r', 'g', 'b', 'orange', 'purple', 'cyan', 'magenta', 'lime', 'gold', 'teal']

        # for i, (s_idx, delta_s, theta) in enumerate(pushes):
        # for i, (start_point, direction) in enumerate(pushes):
        for i, (start_point, end_point, alpha) in enumerate(pushes):
            color = colors_rect[i % len(colors_rect)]
            # start_point = np.array(boundary_points[s_idx])
            # new_start_idx = (s_idx + delta_s) % len(boundary_points)
            # start_point = np.array(boundary_points[new_start_idx])
            # Transform theta to absolute angle
            # alpha = self.transform_theta_to_absolute(theta, start_point)
            # direction = np.array([np.cos(alpha), np.sin(alpha)])
            # alpha = np.arctan2(direction[1], direction[0])
            normal = np.array([-np.sin(alpha), np.cos(alpha)])

            rect_start = start_point
            # t_max = self._compute_ray_box_intersection(start_point, direction)
            rect_end = end_point

            rect = np.array([
                rect_start + normal * (shovel_width / 2),
                rect_end + normal * (shovel_width / 2),
                rect_end - normal * (shovel_width / 2),
                rect_start - normal * (shovel_width / 2),
                rect_start + normal * (shovel_width / 2)
            ])

            # Plot the rectangle
            plt.plot(rect[:, 0], rect[:, 1], color=color, linewidth=2, label=f"Push {i+1}")
            plt.fill(rect[:, 0], rect[:, 1], color=color, alpha=0.2)
            plt.scatter(*start_point, color='red', s=30)  # Push origin

        plt.title(title)
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.xlim(0, 1)
        # plt.ylim(0, 1)
        plt.gca().set_aspect("equal")
        plt.legend()
        plt.show()

    # Additional methods from original (drawing, etc.) can be added here if needed
