#!/usr/bin/env python3
"""
Script to convert elevation.csv into a 2D elevation matrix using a simplified approach.

The approach:
1. Load CSV data with x, y, elevation columns
2. Create pivot table: df.pivot_table(index="y", columns="x", values="elevation")
3. Create meshgrid and extract Z values
4. Define polygon corners and transform coordinates
5. Create polygon mask and clip the data
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.path import Path
from pyproj import Transformer
import os
import sys
import importlib.util

# Add the earth_moving path to import the environment backend
sys.path.append('/home/fedeoli/Documents/Work/Projects/pushing_ws/earth_moving')
from earth_moving.ral.environment.environment_beavers_backend import BeaversEnvironmentBackend

def main():
    """Main function to process DEM data using the simplified approach."""
    
    # Load the CSV file
    csv_path = "/home/fedeoli/Documents/Work/Projects/pushing_ws/earth_moving/earth_moving/utils/elevation.csv"
    df = pd.read_csv(csv_path)
    
    # Assign column names (assuming 3 columns: x, y, elevation)
    df.columns = ["x", "y", "elevation"]
    
    # Create pivot table
    pivot = df.pivot_table(index="y", columns="x", values="elevation")
    
    # Create meshgrid
    x = np.sort(df["x"].unique())
    y = np.sort(df["y"].unique())
    X, Y = np.meshgrid(x, y)
    Z = pivot.values
    
    # Define square corners in lat/lon (EPSG:4326) - Top-left quarter of original map
    # Original full map corners:
    # NW = (-71.0033967, 42.4345814)
    # SW = (-71.0034071, 42.4328122)
    # SE = (-71.0010287, 42.4328093)
    # NE = (-71.0009986, 42.4345665)
    
    # Calculate midpoints for top-left quarter
    # Longitude midpoint: (-71.0033967 + -71.0009986) / 2 = -71.0021977
    # Latitude midpoint: (42.4345814 + 42.4328122) / 2 = 42.4336968
    
    NW = (-71.0033967, 42.4345814)  # Keep original NW corner
    SW = (-71.0034071, 42.4336968)  # Original SW longitude, midpoint latitude  
    SE = (-71.0021977, 42.4336968)  # Midpoint longitude, midpoint latitude
    NE = (-71.0021977, 42.4345814)  # Midpoint longitude, original N latitude
    
    # Order the corners around the square (clockwise or CCW)
    square = [NW, SW, SE, NE]
    
    # Transform coordinates
    tf = Transformer.from_crs("EPSG:4326", "EPSG:2249", always_xy=True)
    square_xy = [tf.transform(lon, lat) for lon, lat in square]
    poly = Path(square_xy)
    
    # Create grid mask
    XY = np.c_[X.ravel(), Y.ravel()]
    inside_grid = poly.contains_points(XY).reshape(X.shape)
    
    # Create land mask (remove water/below sea level)
    Z_land = np.where(Z > 0, Z, -1.0)
    
    # Apply both land mask and polygon mask
    Z_clipped = np.where(inside_grid, Z_land, -1.0)
    
    # Show WITHOUT land mask first (to confirm overlap)
    Z_clip_debug = np.where(inside_grid, Z, -1.0)
    
    # Rescale clipped values from 0 to 1 (using land-only data)
    valid_mask = Z_clipped >= 0.0
    if np.any(valid_mask):
        valid_values = Z_clipped[valid_mask]
        min_val = np.min(valid_values)
        max_val = np.max(valid_values)
        
        print(f"Land-only elevation range: {min_val:.2f} to {max_val:.2f}")
        
        # Create rescaled version
        Z_clip_rescaled = Z_clipped.copy()
        if max_val > min_val:  # Avoid division by zero
            Z_clip_rescaled[valid_mask] = (valid_values - min_val) / (max_val - min_val)
        else:
            Z_clip_rescaled[valid_mask] = 0.5  # If all values are the same, set to middle value
            
        # Keep -1.0 for invalid areas
        Z_clip_rescaled[~valid_mask] = -1.0
            
        print(f"Rescaled elevation range: {np.min(Z_clip_rescaled[valid_mask]):.2f} to {np.max(Z_clip_rescaled[valid_mask]):.2f}")
    else:
        Z_clip_rescaled = Z_clipped.copy()
        print("No valid land elevation data found in clipped region")
    
    # Get bounding box info for reference
    xs = [p[0] for p in square_xy]
    ys = [p[1] for p in square_xy]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    
    print(f"Original data shape: {Z.shape}")
    print(f"Data ranges - x: [{df['x'].min():.2f}, {df['x'].max():.2f}], y: [{df['y'].min():.2f}, {df['y'].max():.2f}]")
    print(f"Square bounds - x: [{xmin:.2f}, {xmax:.2f}], y: [{ymin:.2f}, {ymax:.2f}]")
    print(f"Points inside polygon: {np.sum(inside_grid)} / {inside_grid.size}")
    
    # Create visualization
    plt.figure(figsize=(15, 5))
    
    # Original data
    plt.subplot(1, 3, 1)
    plt.imshow(Z, extent=[X.min(), X.max(), Y.min(), Y.max()], origin='lower', cmap='terrain')
    plt.title('Original DEM')
    plt.colorbar(label='Elevation (m)')
    
    # Polygon mask
    plt.subplot(1, 3, 2)
    plt.imshow(inside_grid, extent=[X.min(), X.max(), Y.min(), Y.max()], origin='lower', cmap='gray')
    plt.title('Polygon Mask')
    
    # Clipped data (rescaled 0-1) - cropped to polygon bounds
    plt.subplot(1, 3, 3)
    plt.imshow(Z_clip_rescaled, extent=[X.min(), X.max(), Y.min(), Y.max()], origin='lower', cmap='terrain')
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.title('Clipped DEM (Rescaled 0-1)')
    plt.colorbar(label='Normalized Elevation')
    
    plt.tight_layout()
    plt.show()
    
    # Save the clipped data
    output_dir = "earth_moving/clutter/csv_analysis/output" 
    os.makedirs(output_dir, exist_ok=True)
    
    # Create coordinate arrays that match the clipped region bounds
    # Find indices within the bounding box
    x_mask = (x >= xmin) & (x <= xmax)
    y_mask = (y >= ymin) & (y <= ymax)
    
    x_clipped = x[x_mask]
    y_clipped = y[y_mask]
    X_clipped, Y_clipped = np.meshgrid(x_clipped, y_clipped)
    
    # Extract the corresponding clipped elevation data
    y_indices = np.where(y_mask)[0]
    x_indices = np.where(x_mask)[0]
    Z_cropped = Z_clip_rescaled[np.ix_(y_indices, x_indices)]

    # Save rescaled clipped elevation matrix (0-1 range) - cropped to actual bounds
    np.save(f"{output_dir}/elevation.npy", Z_cropped)
    
    # Save corresponding coordinates (cropped to match the elevation data)
    np.save(f"{output_dir}/X_coordinates.npy", X_clipped)
    np.save(f"{output_dir}/Y_coordinates.npy", Y_clipped)
    
    # Define tree positions (longitude, latitude) for vegetation clusters
    trees = [
        (-71.0023732, 42.4342306),
        (-71.0022981, 42.4342667),
        (-71.0021906, 42.4342576),
        (-71.0023338, 42.4341820),
        (-71.0022339, 42.4341919),
        (-71.0023810, 42.4341250),
        (-71.0023230, 42.4340780),
        (-71.0022175, 42.4341122),
        (-71.0022483, 42.4340604),
        (-71.0020275, 42.4342362),
        (-71.0019681, 42.4341905),
        (-71.0020670, 42.4341710),
        (-71.0019821, 42.4341208),
        (-71.0020780, 42.4341045),
        (-71.0020140, 42.4340533),
        (-71.0021279, 42.4340349),
        (-71.0020593, 42.4340025),
        (-71.0019668, 42.4339677),
        (-71.0021543, 42.4339098),
        (-71.0020949, 42.4338566),
        (-71.0020142, 42.4338449),
        (-71.0020849, 42.4337932),
        (-71.0019712, 42.4337882),
        (-71.0020076, 42.4337352),
        (-71.0021108, 42.4337290),
        (-71.0022022, 42.4337216),
        (-71.0020904, 42.4336690),
        (-71.0019915, 42.4336875),
        (-71.0018813, 42.4336161),
        (-71.0018026, 42.4335459),
        (-71.0019163, 42.4335420),
        (-71.0022453, 42.4335938),
        (-71.0021175, 42.4335351),
        (-71.0022228, 42.4335263),
        (-71.0023990, 42.4335306),
        (-71.0022980, 42.4334734),
        (-71.0021420, 42.4334536),
        (-71.0021558, 42.4334090),
        (-71.0022649, 42.4334137),
        (-71.0024246, 42.4334385),
        (-71.0023647, 42.4333808),
        (-71.0022889, 42.4332805),
        (-71.0021808, 42.4332686),
        (-71.0020813, 42.4332633),
        (-71.0020588, 42.4331979),
        (-71.0020843, 42.4331193),
        (-71.0021518, 42.4331848),
        (-71.0021964, 42.4331169),
        (-71.0022206, 42.4332092),
        (-71.0023061, 42.4331649),
        (-71.0023650, 42.4332255)
    ]
    # trees = []
    print(f"\nAdding vegetation clusters using BeaversEnvironmentBackend...")
    
    # Add vegetation clusters to the rescaled matrix using environment backend
    Z_with_vegetation, env_backend = add_vegetation_clusters_with_environment_backend(
        Z_clip_rescaled, X, Y, trees,         
        cluster_sigma=1e0,
        cluster_radius=5
    )
    
    # Create enhanced visualization including vegetation
    plt.figure(figsize=(20, 5))
    
    # Original clipped data
    plt.subplot(1, 4, 1)
    plt.imshow(Z_clip_rescaled, extent=[X.min(), X.max(), Y.min(), Y.max()], 
               origin='lower', cmap='terrain')
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.title('Original Clipped DEM')
    plt.colorbar(label='Elevation')
    
    # Data with vegetation
    plt.subplot(1, 4, 2)
    plt.imshow(Z_with_vegetation, extent=[X.min(), X.max(), Y.min(), Y.max()], 
               origin='lower', cmap='terrain')
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.title('DEM with Vegetation Clusters')
    plt.colorbar(label='Elevation + Vegetation')
    
    # Difference (vegetation only)
    vegetation_diff = Z_with_vegetation - Z_clip_rescaled
    plt.subplot(1, 4, 3)
    plt.imshow(vegetation_diff, extent=[X.min(), X.max(), Y.min(), Y.max()], 
               origin='lower', cmap='Greens')
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.title('Vegetation Layer Only')
    plt.colorbar(label='Vegetation Height')
    
    # Tree positions overlay
    plt.subplot(1, 4, 4)
    plt.imshow(Z_clip_rescaled, extent=[X.min(), X.max(), Y.min(), Y.max()], 
               origin='lower', cmap='terrain', alpha=0.7)
    
    # Transform and plot tree positions
    tf = Transformer.from_crs("EPSG:4326", "EPSG:2249", always_xy=True)
    tree_coords_xy = [tf.transform(lon, lat) for lon, lat in trees]
    tree_x_coords = [coord[0] for coord in tree_coords_xy]
    tree_y_coords = [coord[1] for coord in tree_coords_xy]
    plt.scatter(tree_x_coords, tree_y_coords, c='red', s=50, alpha=0.8, marker='x')
    
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.title('Tree Positions on DEM')
    plt.colorbar(label='Elevation')
    
    plt.tight_layout()
    plt.show()
    
    # Extract the vegetation-enhanced data for the cropped region
    Z_vegetation_cropped = Z_with_vegetation[np.ix_(y_indices, x_indices)]
    
    # Save both original and vegetation-enhanced versions
    np.save(f"{output_dir}/elevation.npy", Z_cropped)
    np.save(f"{output_dir}/elevation_with_vegetation.npy", Z_vegetation_cropped)
    
    print(f"\nSaved cropped clipped data to {output_dir}:")
    print(f"- elevation.npy: {Z_cropped.shape} (values 0-1, cropped to polygon bounds)")
    print(f"- elevation_with_vegetation.npy: {Z_vegetation_cropped.shape} (with vegetation clusters)")
    print(f"- X_coordinates.npy: {X_clipped.shape} (cropped coordinates)")
    print(f"- Y_coordinates.npy: {Y_clipped.shape} (cropped coordinates)")
    print(f"- Coordinate ranges: x[{x_clipped.min():.2f}, {x_clipped.max():.2f}], y[{y_clipped.min():.2f}, {y_clipped.max():.2f}]")
    print(f"- Added vegetation using BeaversEnvironmentBackend with {len(trees)} tree positions")
    print(f"- Vegetation parameters: radius={env_backend._vegetation_cluster_radius_range}, sigma={env_backend._vegetation_cluster_sigma}")

    return df, pivot, X, Y, Z, Z_clip_debug, Z_clip_rescaled, inside_grid, Z_with_vegetation, env_backend

def add_vegetation_clusters_with_environment_backend(Z_matrix, X_coords, Y_coords, tree_positions, cluster_sigma, cluster_radius=3):
    """
    Add vegetation clusters (trees) using the BeaversEnvironmentBackend generate_cluster method.
    
    This function creates a temporary environment backend instance, loads the DEM data,
    and uses the proper vegetation generation system to add realistic vegetation clusters.
    
    Parameters:
    -----------
    Z_matrix : numpy.ndarray
        The elevation matrix to use as base
    X_coords : numpy.ndarray
        X coordinate meshgrid
    Y_coords : numpy.ndarray
        Y coordinate meshgrid  
    tree_positions : list of tuples
        List of (longitude, latitude) coordinates for tree positions
    cluster_radius : int, optional
        Radius of each vegetation cluster in grid cells (default: 3)
        
    Returns:
    --------
    numpy.ndarray
        Modified elevation matrix with vegetation clusters added using the environment backend
    BeaversEnvironmentBackend
        The environment backend instance (for further use if needed)
    """
    # Transform tree coordinates from lat/lon to the same coordinate system
    tf = Transformer.from_crs("EPSG:4326", "EPSG:2249", always_xy=True)
    tree_coords_xy = [tf.transform(lon, lat) for lon, lat in tree_positions]
    
    # Get the coordinate arrays
    x_coords = X_coords[0, :]  # First row contains all x coordinates
    y_coords = Y_coords[:, 0]  # First column contains all y coordinates
    
    # Create environment configuration matching the DEM data
    environment_config = {
        'environment': {
            'map_mode': 'generate',  # We'll override the map with our DEM data
            'width': Z_matrix.shape[1],
            'height': Z_matrix.shape[0],
            'vegetation_quality_range': [0.0, 1.0],            
            'vegetation_cluster_sigma': cluster_sigma,
            'vegetation_cluster_radius_range': [cluster_radius-1, cluster_radius+1],
            'number_vegetation_clusters_init': 0,  # Start with no clusters, we'll add them manually
            'number_vegetation_clusters_max': int(1e3),  # Allow space for tree positions
            'streams_number': 0,  # No streams generation since we have real DEM data
            'streams_width': 1,
            'print': True
        }
    }
    
    # Create environment backend instance
    env_backend = BeaversEnvironmentBackend()
    env_backend.initiate_environment(**environment_config)
    
    # Replace the generated map with our DEM data
    # Note: BeaversEnvironmentBackend uses (width, height) = (x, y) format
    # Our Z_matrix is in (height, width) = (y, x) format, so we need to transpose
    env_backend._map_original = Z_matrix.T.copy()
    env_backend._width = Z_matrix.shape[1] 
    env_backend._height = Z_matrix.shape[0]
    
    print(f"Environment backend initialized with original DEM elevation data")
    print(f"Map dimensions: {env_backend._width} x {env_backend._height}")
    print(f"Elevation range in map: {np.min(env_backend._map_original):.2f} to {np.max(env_backend._map_original):.2f}")
    
    # Convert tree coordinates to grid indices and add clusters
    trees_added = 0
    for tree_x, tree_y in tree_coords_xy:
        # Find the closest grid indices
        x_idx = np.argmin(np.abs(x_coords - tree_x))
        y_idx = np.argmin(np.abs(y_coords - tree_y))
        
        # Check if the position is within the valid area and not in water
        if (0 <= x_idx < env_backend._width and 
            0 <= y_idx < env_backend._height and 
            env_backend._map_original[x_idx, y_idx] >= 0):
            
            # Use the environment backend's generate_cluster method
            # Note: environment backend uses (x, y) coordinates
            env_backend.generate_cluster(x_idx, y_idx, cluster_radius)
            trees_added += 1
        else:
            print(f"Skipping tree at coordinates ({tree_x:.2f}, {tree_y:.2f}) - outside valid area or in water")
    
    print(f"Added {trees_added} vegetation clusters using environment backend")
    
    # Get the modified map and transpose back to our format
    Z_with_vegetation = env_backend._map_original.T.copy()
    
    return Z_with_vegetation, env_backend

if __name__ == "__main__":
    # Run the basic processing
    main()
