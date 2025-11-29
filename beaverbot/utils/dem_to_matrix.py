#!/usr/bin/env python3
"""Module for converting DEM CSV data into elevation matrices for beaver simulation."""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from pyproj import Transformer
from scipy.interpolate import griddata
import os
import json

# Constants
INVALID_MARKER = -1.0 # Marker for invalid/no-data pixels


def load_csv_data(csv_path):
    """Load DEM data from CSV file and standardize column names."""
    # Read CSV with explicit decimal separator to avoid parsing issues
    df = pd.read_csv(csv_path, decimal='.')
    
    if 'X_m' in df.columns and 'Y_m' in df.columns:
        df = df[['X_m', 'Y_m', 'elevation']].copy()
        df.columns = ['x', 'y', 'elevation']
        coordinates_are_projected = True
        print("Using projected coordinates (X_m, Y_m)")
    else:
        if df.shape[1] > 3:
            df = df.iloc[:, :3]
        df.columns = ['x', 'y', 'elevation']
        
        # Auto-detect if coordinates are projected based on value ranges
        # Geographic coordinates (lat/lon) should be in range [-180, 180] and [-90, 90]
        x_range = df['x'].abs().max()
        y_range = df['y'].abs().max()
        
        if x_range > 180 or y_range > 90:
            coordinates_are_projected = True
            print("Detected projected coordinates (x, y values > geographic range)")
        else:
            coordinates_are_projected = False
            print("Detected geographic coordinates (lon, lat)")
    
    print(f"\nData loaded: {len(df)} points")
    print(f"X/Lon range: [{df['x'].min():.6f}, {df['x'].max():.6f}]")
    print(f"Y/Lat range: [{df['y'].min():.6f}, {df['y'].max():.6f}]")
    print(f"Elevation range: [{df['elevation'].min():.2f}, {df['elevation'].max():.2f}] m")
    
    return df, coordinates_are_projected


def transform_coordinates_to_target_crs(df, X, Y, epsg_source, epsg_target):
    """Transform coordinates from source CRS to target CRS (e.g., feet to meters).
    
    Args:
        df: DataFrame with 'x', 'y', 'elevation' columns in source CRS
        X: Meshgrid X array in source CRS
        Y: Meshgrid Y array in source CRS
        epsg_source: Source EPSG code (e.g., 'EPSG:2249' for feet)
        epsg_target: Target EPSG code (e.g., 'EPSG:6483' for meters)
    
    Returns:
        df_transformed: DataFrame with coordinates in target CRS
        X_transformed: Meshgrid X array in target CRS
        Y_transformed: Meshgrid Y array in target CRS
    """
    if epsg_source == epsg_target:
        print(f"Source and target CRS are the same ({epsg_source}), no transformation needed")
        return df.copy(), X, Y
    
    print(f"Transforming coordinates from {epsg_source} to {epsg_target}...")
    
    # Create transformer
    tf = Transformer.from_crs(epsg_source, epsg_target, always_xy=True)
    
    # Transform dataframe coordinates
    df_transformed = df.copy()
    df_transformed['x'], df_transformed['y'] = tf.transform(df['x'].values, df['y'].values)
    
    # Transform meshgrid coordinates
    X_transformed, Y_transformed = tf.transform(X, Y)
    
    print(f"  Coordinates transformed")
    print(f"  New X range: [{X_transformed.min():.2f}, {X_transformed.max():.2f}]")
    print(f"  New Y range: [{Y_transformed.min():.2f}, {Y_transformed.max():.2f}]")
    
    return df_transformed, X_transformed, Y_transformed


def resample_dem_to_target_resolution(df, target_resolution_m=1.0, interpolation_method='linear'):
    """Resample DEM data to a target resolution using interpolation."""
    print(f"Checking resolution...")
    
    x_min, x_max = df['x'].min(), df['x'].max()
    y_min, y_max = df['y'].min(), df['y'].max()
    
    print(f"Original data bounds: x[{x_min:.2f}, {x_max:.2f}], y[{y_min:.2f}, {y_max:.2f}]")
    
    x_unique = np.sort(df['x'].unique())
    y_unique = np.sort(df['y'].unique())
    
    original_x_res = None
    original_y_res = None
    
    if len(x_unique) > 1 and len(y_unique) > 1:
        original_x_res = np.mean(np.diff(x_unique))
        original_y_res = np.mean(np.diff(y_unique))
        print(f"Original resolution: {original_x_res:.2f}m x {original_y_res:.2f}m")
        
        Z = df.pivot(index='y', columns='x', values='elevation').values
        Z[Z < 0.0] = INVALID_MARKER
        
        # Check if resampling is needed (with small tolerance for floating point comparison)
        if (abs(original_x_res - target_resolution_m) < 0.01 and 
            abs(original_y_res - target_resolution_m) < 0.01):
            print(f"  Target resolution ({target_resolution_m}m) matches original resolution - skipping resampling")
            X, Y = np.meshgrid(x_unique, y_unique)            
            return X, Y, Z
    
    print(f"Resampling DEM to {target_resolution_m}m resolution using {interpolation_method} interpolation...")
    
    x_new = np.arange(x_min, x_max + target_resolution_m, target_resolution_m)
    y_new = np.arange(y_min, y_max + target_resolution_m, target_resolution_m)
    
    x_new = x_new[x_new <= x_max]
    y_new = y_new[y_new <= y_max]
    
    X_new, Y_new = np.meshgrid(x_new, y_new)
    
    print(f"Target grid shape: {X_new.shape} ({len(y_new)} x {len(x_new)})")
    print(f"Target resolution: {target_resolution_m:.2f}m x {target_resolution_m:.2f}m")
        
    df_valid = df.copy()
    
    print(f"Using {len(df_valid)} valid points out of {len(df)} total points")
    
    points = df_valid[['x', 'y']].values
    values = df_valid['elevation'].values
    values[values < 0] = INVALID_MARKER
    
    Z_new = griddata(points, values, (X_new, Y_new), method=interpolation_method, fill_value=np.nan)
    
    nan_count = np.sum(np.isnan(Z_new))
    total_pixels = Z_new.size
    print(f"Interpolation complete. {nan_count}/{total_pixels} pixels have no data (NaN)")
    
    return X_new, Y_new, Z_new


def create_polygon_mask(X, Y, polygon_corners, coordinates_are_projected, epsg_geographic, epsg_projected, df=None):
    """Create a boolean mask for the specified polygon region.
    
    Args:
        X, Y: Meshgrid coordinate arrays
        polygon_corners: List of (lon, lat) tuples defining the polygon
        coordinates_are_projected: Whether input data is already projected
        epsg_geographic: EPSG code for geographic CRS (e.g., 'EPSG:4326')
        epsg_projected: EPSG code for projected CRS (e.g., 'EPSG:6483')
        df: DataFrame with DEM data (optional, for coordinate transformation)
    """
    if len(polygon_corners) == 0:
        print("No polygon defined, using entire map extent")
        inside_grid = np.ones(X.shape, dtype=bool)
        xmin, xmax = X.min(), X.max()
        ymin, ymax = Y.min(), Y.max()
        return inside_grid, (xmin, xmax, ymin, ymax), df
    
    print(f"Using polygon with {len(polygon_corners)} corners")
    
    tf = Transformer.from_crs(epsg_geographic, epsg_projected, always_xy=True)
    polygon_xy = [tf.transform(lon, lat) for lon, lat in polygon_corners]
    
    if not coordinates_are_projected and df is not None:
        print("Transforming DEM coordinates to projected system")
        df['x'], df['y'] = tf.transform(df['x'].values, df['y'].values)
    
    poly = Path(polygon_xy)
    XY = np.c_[X.ravel(), Y.ravel()]
    inside_grid = poly.contains_points(XY).reshape(X.shape)
    
    xs = [p[0] for p in polygon_xy]
    ys = [p[1] for p in polygon_xy]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    
    print(f"Polygon bounds: x[{xmin:.2f}, {xmax:.2f}], y[{ymin:.2f}, {ymax:.2f}]")
    print(f"Points inside region: {np.sum(inside_grid)} / {inside_grid.size}")
    
    return inside_grid, (xmin, xmax, ymin, ymax), df


def normalize_elevation_data(Z):
    """Normalize raw elevation data to standard ranges.
    
    Maps elevation data to:
    - Land (positive elevations): [0, 1] normalized range
    - Water (NaN or negative): marked with INVALID_MARKER
    
    This should be applied immediately after loading CSV data, before any
    polygon masking or stream removal.
    
    Args:
        Z: Raw elevation matrix (may contain NaN values)
    
    Returns:
        Z_normalized: Elevation matrix with:
                     - Land elevations normalized to [0, 1]
                     - Water/NaN marked as INVALID_MARKER (-9999.0)
    """
    # Step 1: Handle NaN and negative values (water) - replace with marker
    Z_processed = np.where(np.isnan(Z), INVALID_MARKER, Z)
    
    # Step 2: Identify valid land pixels (finite, positive values)
    valid_land_mask = (Z_processed >= 0) & np.isfinite(Z_processed)
    
    if not np.any(valid_land_mask):
        print("Warning: No valid elevation points found in data")
        return np.full_like(Z, INVALID_MARKER)
    
    # Step 3: Normalize valid land elevations to [0, 1]
    valid_values = Z_processed[valid_land_mask]
    min_elevation = np.min(valid_values)
    max_elevation = np.max(valid_values)
    
    print(f"Normalizing elevation data:")
    print(f"  Original elevation range: [{min_elevation:.2f}, {max_elevation:.2f}] m")
    
    Z_normalized = Z_processed.copy()
    
    if max_elevation > min_elevation:
        Z_normalized[valid_land_mask] = (valid_values - min_elevation) / (max_elevation - min_elevation)
    else:
        # All land at same elevation
        Z_normalized[valid_land_mask] = 0.5
        print(f"  Warning: Uniform elevation detected, setting to 0.5")
        
    # Step 4: Normalize valid water elevations to [-1, 0]
    water_mask = ~valid_land_mask
    if np.any(water_mask):
        water_values = Z_processed[water_mask]
        min_water = np.min(water_values)
        max_water = np.max(water_values)
        
        if max_water > min_water:
            # Linear mapping: [min_water, max_water] → [-1, 0]
            Z_normalized[water_mask] = -1 + (water_values - min_water) / (max_water - min_water)
        else:
            # All water at same elevation
            Z_normalized[water_mask] = -0.5
    
    # Summary statistics
    n_valid = np.sum(valid_land_mask)
    n_invalid = np.sum(~valid_land_mask)
    total = Z_normalized.size
    
    print(f"  Valid land pixels: {n_valid:,} ({100*n_valid/total:.1f}%)")
    print(f"  Water/invalid pixels: {n_invalid:,} ({100*n_invalid/total:.1f}%)")
    print(f"  Normalized range: [0.00, 1.00]")
    
    return Z_normalized


def apply_global_stream_threshold(Z, threshold_percentile=9):
    """Apply percentile-based stream removal globally (before polygon masking).
    
    Uses percentile thresholding to identify and mark low-elevation areas (streams)
    as water based on the full dataset. Should be applied after normalization
    but before polygon definition.
    
    Args:
        Z: Normalized elevation matrix (from normalize_elevation_data)
        threshold_percentile: Percentile threshold for stream removal (0-100)
                            0 = skip stream removal
    
    Returns:
        Z_processed: Elevation matrix with streams marked as -1
    """
    Z_processed = Z.copy()
    
    # Identify valid land pixels (normalized values [0, 1])
    valid_land_mask = (Z_processed >= 0) & np.isfinite(Z_processed)
    
    if not np.any(valid_land_mask):
        print("Warning: No valid elevation points found in data")
        return Z_processed
    
    # Stream removal (optional, based on percentile of full dataset)
    if threshold_percentile > 0:
        valid_elevations = Z_processed[valid_land_mask]
        threshold = np.percentile(valid_elevations, threshold_percentile)
        print(f"\nApplying global stream threshold:")
        print(f"  {threshold_percentile}th percentile = {threshold:.4f} (normalized)")
        
        # Mark low elevations as water and remap to [-1, 0] range
        stream_mask = valid_land_mask & (Z_processed <= threshold)
        
        # Remap water values: lowest elevation → -1, threshold → 0
        if np.any(stream_mask):
            water_elevations = Z_processed[stream_mask]
            min_water = np.min(water_elevations)
            max_water = threshold
            
            if max_water > min_water:
                # Linear mapping: [min_water, threshold] → [-1, 0]
                Z_processed[stream_mask] = -1 + (water_elevations - min_water) / (max_water - min_water)
            else:
                # All water at same elevation
                Z_processed[stream_mask] = -0.5
        
        removed_pixels = np.sum(stream_mask)
        total_valid = np.sum(valid_land_mask)
        print(f"  Removed {removed_pixels:,} / {total_valid:,} pixels ({100*removed_pixels/total_valid:.1f}%)")
    else:
        print("\nStream removal: skipped (threshold_percentile = 0)")
    
    return Z_processed


def crop_to_bounds(Z, X, Y, bounds):
    """Crop elevation matrix and coordinates to specified bounds."""
    xmin, xmax, ymin, ymax = bounds
    
    x_coords = X[0, :]
    y_coords = Y[:, 0]
    
    x_mask = (x_coords >= xmin) & (x_coords <= xmax)
    y_mask = (y_coords >= ymin) & (y_coords <= ymax)
    
    x_clipped = x_coords[x_mask]
    y_clipped = y_coords[y_mask]
    X_clipped, Y_clipped = np.meshgrid(x_clipped, y_clipped)
    
    y_indices = np.where(y_mask)[0]
    x_indices = np.where(x_mask)[0]
    Z_cropped = Z[np.ix_(y_indices, x_indices)]
    
    return Z_cropped, X_clipped, Y_clipped, x_clipped, y_clipped


def transform_food_cache_coordinates(food_cache_latlon, x_clipped, y_clipped, epsg_geographic, epsg_projected):
    """Transform food cache coordinates and find matrix indices.
    
    Args:
        food_cache_latlon: Tuple of (latitude, longitude) for food cache
        x_clipped, y_clipped: Clipped coordinate arrays
        epsg_geographic: EPSG code for geographic CRS (e.g., 'EPSG:4326')
        epsg_projected: EPSG code for projected CRS (e.g., 'EPSG:6483')
    """
    tf = Transformer.from_crs(epsg_geographic, epsg_projected, always_xy=True)
    food_cache_x, food_cache_y = tf.transform(food_cache_latlon[1], food_cache_latlon[0])
    
    food_cache_x_idx = np.argmin(np.abs(x_clipped - food_cache_x))
    food_cache_y_idx = np.argmin(np.abs(y_clipped - food_cache_y))
    
    food_cache_in_bounds = (
        x_clipped.min() <= food_cache_x <= x_clipped.max() and
        y_clipped.min() <= food_cache_y <= y_clipped.max()
    )
    
    print(f"Food cache coordinates: ({food_cache_x:.2f}, {food_cache_y:.2f})")
    print(f"Food cache matrix indices: ({food_cache_x_idx}, {food_cache_y_idx})")
    print(f"Food cache within bounds: {food_cache_in_bounds}")
    
    return (food_cache_x, food_cache_y), (food_cache_x_idx, food_cache_y_idx), food_cache_in_bounds


def save_dem_outputs(output_dir, Z_cropped, X_clipped, Y_clipped, 
                     x_clipped, y_clipped, target_resolution_m, interpolation_method,
                     food_cache_latlon, food_cache_proj, food_cache_idx, food_cache_in_bounds,
                     epsg_projected):
    """Save all DEM processing outputs and metadata.
    
    Args:
        output_dir: Directory to save outputs
        Z_cropped: Processed elevation matrix
        X_clipped, Y_clipped: Clipped coordinate meshgrids
        x_clipped, y_clipped: Clipped coordinate arrays
        target_resolution_m: Target resolution in meters
        interpolation_method: Interpolation method used
        food_cache_latlon: Food cache (lat, lon) coordinates
        food_cache_proj: Food cache projected coordinates
        food_cache_idx: Food cache matrix indices
        food_cache_in_bounds: Whether food cache is within bounds
        epsg_projected: EPSG code for projected CRS
    """
    os.makedirs(output_dir, exist_ok=True)
    
    np.save(f"{output_dir}/elevation.npy", Z_cropped)
    np.save(f"{output_dir}/X_coordinates.npy", X_clipped)
    np.save(f"{output_dir}/Y_coordinates.npy", Y_clipped)
    
    metadata = {
        'target_resolution_m': target_resolution_m,
        'interpolation_method': interpolation_method,
        'matrix_shape': list(Z_cropped.shape),
        'coordinate_ranges': {
            'x_min': float(x_clipped.min()),
            'x_max': float(x_clipped.max()),
            'y_min': float(y_clipped.min()),
            'y_max': float(y_clipped.max())
        },
        'pixel_area_m2': target_resolution_m * target_resolution_m,
        'total_area_m2': float((x_clipped.max() - x_clipped.min()) * (y_clipped.max() - y_clipped.min())),
        'coordinate_system': str(epsg_projected),
        'elevation_normalization': 'Values normalized 0-1, invalid areas marked as -1.0',
        'food_cache': {
            'original_coordinates': {
                'latitude': float(food_cache_latlon[0]),
                'longitude': float(food_cache_latlon[1])
            },
            'projected_coordinates': {
                'x': float(food_cache_proj[0]),
                'y': float(food_cache_proj[1])
            },
            'matrix_indices': {
                'x_index': int(food_cache_idx[0]),
                'y_index': int(food_cache_idx[1])
            },
            'within_bounds': bool(food_cache_in_bounds)
        },
        'processing_timestamp': pd.Timestamp.now().isoformat()
    }
    
    with open(f"{output_dir}/processing_metadata.json", 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"\n  Saved to {output_dir}:")
    print(f"  - elevation.npy: {Z_cropped.shape}")
    print(f"  - X_coordinates.npy: {X_clipped.shape}")
    print(f"  - Y_coordinates.npy: {Y_clipped.shape}")
    print(f"  - processing_metadata.json")
