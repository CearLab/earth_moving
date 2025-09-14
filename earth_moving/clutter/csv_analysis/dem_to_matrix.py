#!/usr/bin/env python3
"""
Script to convert exported_dem.csv into a 2D elevation matrix.

The CSV file contains longitude, latitude, and elevation data.
This script converts it into a 2D numpy array where:
- Rows correspond to latitude values (y-axis)
- Columns correspond to longitude values (x-axis)
- Values are elevation measurements
"""


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Add geopandas and shapely for coordinate transformation
import geopandas as gpd
from shapely.geometry import Point
from pyproj import Proj

def convert_latlon_to_xy(df, lon_col='x', lat_col='y', crs_out="EPSG:32619"):
    """
    Convert latitude/longitude columns to projected x/y using geopandas.
    Default output CRS is UTM zone 19N (EPSG:32619), suitable for Boston.
    Adds 'x_proj' and 'y_proj' columns to the DataFrame.
    """
    gdf = gpd.GeoDataFrame(
        df,
        geometry=[Point(lon, lat) for lon, lat in zip(df[lon_col], df[lat_col])],
        crs="EPSG:4326"
    )
    gdf = gdf.to_crs(crs_out)
    df['x_proj'] = gdf.geometry.x
    df['y_proj'] = gdf.geometry.y
    return df

def project_latlon_to_xy(df, lon_col='x', lat_col='y'):
    """Project latitude/longitude columns to x/y using geopandas. Adds x_proj and y_proj columns."""
    
    # Define UTM projection (replace 'zone' with appropriate UTM zone, e.g., 10 for parts of North America)
    utm = Proj(proj='utm', zone=10, ellps='WGS84')

    # Convert lat/lon to x,y
    df['x_proj'], df['y_proj'] = utm(df[lon_col].values, df[lat_col].values)
    return df

def load_dem_data(csv_path, x_col='x', y_col='y', elev_col='elevation'):
    """Load DEM data from CSV file."""
    print(f"Loading data from {csv_path}...")
    
    # Load the CSV file
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points")
    print(f"Columns: {list(df.columns)}")
    
    # Display basic statistics
    print("\nData ranges:")
    print(f"{x_col}: {df[x_col].min():.6f} to {df[x_col].max():.6f}")
    print(f"{y_col}: {df[y_col].min():.6f} to {df[y_col].max():.6f}")
    print(f"{elev_col}: {df[elev_col].min():.2f} to {df[elev_col].max():.2f}")
    
    nan_elev_count = df[elev_col].isna().sum()
    total_elev_count = len(df)
    print(f"\nNumber of NaN {elev_col} values: {nan_elev_count} / {total_elev_count} ({100 * nan_elev_count / total_elev_count:.2f}%)")
    
    return df

def create_elevation_matrix(df, x_col='x', y_col='y', scale=1):
    """
    Bin projected x/y data onto a regular grid using average dx/dy, assigning each measurement to the closest grid cell.
    If multiple points fall in a cell, use the mean elevation.
    """
    print("Binning projected x/y data onto a regular grid...")
    x_vals = df[x_col].values
    y_vals = df[y_col].values

    print('\nData shapes:')
    print(f"x_vals shape: {x_vals.shape}")
    print(f"y_vals shape: {y_vals.shape}")
    
    # dx_raw = abs(np.mean(np.diff(np.sort(np.unique(x_vals)))))
    # dy_raw = abs(np.mean(np.diff(np.sort(np.unique(y_vals)))))
    dx_raw = 1e-2
    dy_raw = 1e-2
    
    dx = scale * dx_raw
    dy = scale * dy_raw    
    x_min, x_max = x_vals.min(), x_vals.max()
    y_min, y_max = y_vals.min(), y_vals.max()
    n_x = int(np.ceil(abs((x_max - x_min) / dx)) + 1)
    n_y = int(np.ceil(abs((y_max - y_min) / dy)) + 1)
    x_grid = x_min + np.arange(n_x) * dx
    y_grid = y_min + np.arange(n_y) * dy
    
    print(f"\nGrid parameters:")
    print(f"Estimated grid step: dx={dx:.6f} m (scale={scale}), dy={dy:.6f} m (scale={scale})")
    print(f"x_min: {x_min:.6f}, x_max: {x_max:.6f}")
    print(f"y_min: {y_min:.6f}, y_max: {y_max:.6f}")
    print(f"n_x (columns): {n_x}, n_y (rows): {n_y}")
    
    # Shift grids so that they start from 0
    x_grid_shifted = x_grid - x_min
    y_grid_shifted = y_grid - y_min
    
    print(f"\nGrid shape:")
    print(f"Grid shape: {len(y_grid_shifted)} rows x {len(x_grid_shifted)} cols (shifted to 0,0)")    
    print(f"x_grid: min={x_grid.min():.6f}, max={x_grid.max():.6f}")
    print(f"y_grid: min={y_grid.min():.6f}, max={y_grid.max():.6f}")
    print(f"x_grid_shifted: min={x_grid_shifted.min():.6f}, max={x_grid_shifted.max():.6f}")
    print(f"y_grid_shifted: min={y_grid_shifted.min():.6f}, max={y_grid_shifted.max():.6f}")
    
    # Assign each point to the closest grid cell (using shifted values)
    x_idx = np.searchsorted(x_grid, x_vals, side='left')
    y_idx = np.searchsorted(y_grid, y_vals, side='left')
    x_idx = np.clip(x_idx, 0, len(x_grid_shifted)-1)
    y_idx = np.clip(y_idx, 0, len(y_grid_shifted)-1)    
    
    print(f"\nIndexing:")
    print(f"x_idx: min={x_idx.min()}, max={x_idx.max()}, unique={len(np.unique(x_idx))}")
    print(f"y_idx: min={y_idx.min()}, max={y_idx.max()}, unique={len(np.unique(y_idx))}")
    
    # Bin elevations
    elevation_matrix = np.full((len(y_grid_shifted), len(x_grid_shifted)), np.nan)
    count_matrix = np.zeros((len(y_grid_shifted), len(x_grid_shifted)), dtype=int)
    sum_matrix = np.zeros((len(y_grid_shifted), len(x_grid_shifted)), dtype=float)
    for xi, yi, elev in zip(x_idx, y_idx, df['elevation'].values):        
        if not np.isnan(elev):
            sum_matrix[yi, xi] += elev
            count_matrix[yi, xi] += 1
    mask = count_matrix > 0
    elevation_matrix[mask] = sum_matrix[mask] / count_matrix[mask]
    nan_count = np.isnan(elevation_matrix).sum()
    total_points = elevation_matrix.size
    
    print(f"\nBinned elevation matrix")
    print(f"Matrix shape: {elevation_matrix.shape}")
    print(f"NaN values: {nan_count} / {total_points} ({100*nan_count/total_points:.1f}%)")
    return elevation_matrix, x_grid_shifted, y_grid_shifted

def save_matrix(elevation_matrix, lon_coords, lat_coords, output_dir="outputs"):
    """Save the elevation matrix and coordinates to files."""
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)
    
    # Save as numpy arrays
    np.save(output_path / "elevation_matrix.npy", elevation_matrix)
    np.save(output_path / "longitude_coords.npy", lon_coords)
    np.save(output_path / "latitude_coords.npy", lat_coords)
    
    # Save as CSV (for smaller matrices)
    if elevation_matrix.size < 10000000:  # Less than 10M elements
        np.savetxt(output_path / "elevation_matrix.csv", elevation_matrix, delimiter=',')
    
    print(f"Matrix saved to {output_path}/")
    return output_path

def visualize_dem(elevation_matrix, x_coords, y_coords, output_dir="outputs"):
    """Create visualizations of the DEM using projected x/y coordinates."""
    plt.figure(figsize=(15, 10))

    extent = [x_coords.min(), x_coords.max(), y_coords.min(), y_coords.max()]

    plt.subplot(2, 2, 1)
    plt.imshow(elevation_matrix, extent=extent, origin='lower', cmap='gray', aspect='auto')
    plt.colorbar(label='Elevation (m)')
    plt.title('DEM Elevation Map (UTM)')
    plt.xlabel('X (meters, UTM)')
    plt.ylabel('Y (meters, UTM)')

    plt.subplot(2, 2, 2)
    plt.hist(elevation_matrix[~np.isnan(elevation_matrix)].flatten(), bins=50, alpha=0.7)
    plt.title('Elevation Distribution')
    plt.xlabel('Elevation (m)')
    plt.ylabel('Frequency')

    plt.subplot(2, 2, 3)
    coverage = ~np.isnan(elevation_matrix)
    plt.imshow(coverage, extent=extent, origin='lower', cmap='gray', aspect='auto')
    plt.title('Data Coverage (white = data, black = no data)')
    plt.xlabel('X (meters, UTM)')
    plt.ylabel('Y (meters, UTM)')

    plt.subplot(2, 2, 4)
    x_grid, y_grid = np.meshgrid(x_coords, y_coords)
    contours = plt.contour(x_grid, y_grid, elevation_matrix, levels=20)
    plt.clabel(contours, inline=True, fontsize=8)
    plt.title('Elevation Contours (UTM)')
    plt.xlabel('X (meters, UTM)')
    plt.ylabel('Y (meters, UTM)')

    plt.tight_layout()

    output_path = Path(output_dir)
    plt.savefig(output_path / "dem_visualization.png", dpi=300, bbox_inches='tight')
    plt.show()

    print(f"Visualization saved to {output_path}/dem_visualization.png")
    
def scatter_dem(df, output_dir="outputs", x_col='x_relative', y_col='y_relative', elev_col='elevation'):
    """Scatter plot of DEM points colored by elevation."""    
    plt.figure(figsize=(10, 8))
    sc = plt.scatter(df[x_col], df[y_col], c=df[elev_col], cmap='viridis', s=5, alpha=0.7)
    plt.colorbar(sc, label='Elevation (m)')
    plt.xlabel('X (meters, relative)')
    plt.ylabel('Y (meters, relative)')
    plt.title('DEM Scatter Plot (Colored by Elevation)')
    plt.tight_layout()
    output_path = Path(output_dir)
    plt.savefig(output_path / "dem_scatter.png", dpi=300, bbox_inches='tight')
    # plt.show()
    print(f"Scatter plot saved to {output_path}/dem_scatter.png")

def main():
    """Main function to process the DEM data."""
    
    # Path to the CSV file
    csv_path = "/home/fedeoli/Documents/Work/Projects/pushing_ws/earth_moving/earth_moving/utils/exported_dem_02.csv"
    output_dir = "earth_moving/clutter/csv_analysis/output"
    
    try:

        # Load data
        df = load_dem_data(csv_path, x_col='x_relative', y_col='y_relative', elev_col='elevation')

        # Convert lat/lon to projected x/y coordinates        
        # df = project_latlon_to_xy(df, lon_col='x', lat_col='y')
        # df = convert_latlon_to_xy(df, lon_col='x', lat_col='y')
        
        # Scatter plot of raw points
        scatter_dem(df, output_dir, x_col='x_relative', y_col='y_relative', elev_col='elevation')
        # scatter_dem(df, output_dir, x_col='x_proj', y_col='y_proj', elev_col='elevation')

        print("\nMatrix transformation complete!")        

        return 0
        
    except Exception as e:
        print(f"Error processing DEM data: {e}")
        return None, None, None

if __name__ == "__main__":
    main()
