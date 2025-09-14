#!/usr/bin/env python3
"""
Simple CSV to matrix converter using only numpy and built-in libraries.
This version is more memory efficient for large datasets.
"""

import numpy as np
import csv
from collections import defaultdict

def load_dem_simple(csv_path):
    """Load DEM data using basic CSV reader for memory efficiency."""
    print(f"Loading data from {csv_path}...")
    
    data = []
    with open(csv_path, 'r') as file:
        reader = csv.DictReader(file)
        for i, row in enumerate(reader):
            if i % 1000000 == 0:  # Progress indicator
                print(f"Processed {i:,} rows...")
            
            data.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'elevation': float(row['elevation'])
            })
    
    print(f"Loaded {len(data):,} data points")
    return data

def create_matrix_simple(data):
    """Create elevation matrix using simple approach."""
    print("Analyzing coordinate structure...")
    
    # Extract unique coordinates
    x_coords = sorted(set(point['x'] for point in data))
    y_coords = sorted(set(point['y'] for point in data))
    
    print(f"Grid dimensions: {len(y_coords)} x {len(x_coords)}")
    
    # Create coordinate to index mappings
    x_to_idx = {x: i for i, x in enumerate(x_coords)}
    y_to_idx = {y: i for i, y in enumerate(y_coords)}
    
    # Initialize matrix with NaN
    elevation_matrix = np.full((len(y_coords), len(x_coords)), np.nan)
    
    # Fill matrix
    print("Filling elevation matrix...")
    for i, point in enumerate(data):
        if i % 1000000 == 0:
            print(f"Processed {i:,} points...")
        
        row_idx = y_to_idx[point['y']]
        col_idx = x_to_idx[point['x']]
        elevation_matrix[row_idx, col_idx] = point['elevation']
    
    return elevation_matrix, np.array(x_coords), np.array(y_coords)

def save_matrix_simple(elevation_matrix, x_coords, y_coords, output_prefix="dem_matrix"):
    """Save matrix and coordinates."""
    print("Saving matrix and coordinates...")
    
    # Save as numpy binary files (most efficient)
    np.save(f"{output_prefix}_elevation.npy", elevation_matrix)
    np.save(f"{output_prefix}_longitude.npy", x_coords)
    np.save(f"{output_prefix}_latitude.npy", y_coords)
    
    # Save basic info
    with open(f"{output_prefix}_info.txt", 'w') as f:
        f.write(f"Matrix shape: {elevation_matrix.shape}\n")
        f.write(f"Longitude range: {x_coords.min():.8f} to {x_coords.max():.8f}\n")
        f.write(f"Latitude range: {y_coords.min():.8f} to {y_coords.max():.8f}\n")
        f.write(f"Elevation range: {np.nanmin(elevation_matrix):.2f} to {np.nanmax(elevation_matrix):.2f}\n")
        f.write(f"NaN values: {np.isnan(elevation_matrix).sum()} / {elevation_matrix.size}\n")
    
    print(f"Files saved with prefix: {output_prefix}")

def sample_large_dataset(csv_path, sample_size=100000, output_file="dem_sample.csv"):
    """Create a smaller sample for testing."""
    print(f"Creating sample of {sample_size:,} points...")
    
    # Count total lines
    with open(csv_path, 'r') as f:
        total_lines = sum(1 for _ in f) - 1  # Subtract header
    
    # Calculate sampling interval
    interval = max(1, total_lines // sample_size)
    
    with open(csv_path, 'r') as infile, open(output_file, 'w') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)
        
        # Write header
        header = next(reader)
        writer.writerow(header)
        
        # Sample data
        for i, row in enumerate(reader):
            if i % interval == 0:
                writer.writerow(row)
    
    print(f"Sample saved to {output_file}")

def main():
    """Main processing function."""
    csv_path = "/home/fedeoli/Documents/Work/Projects/pushing_ws/earth_moving/earth_moving/utils/exported_dem.csv"
    
    print("DEM CSV to Matrix Converter")
    print("===========================")
    
    # For testing with large datasets, create a sample first
    create_sample = input("Create a sample first? (y/n): ").lower().strip() == 'y'
    
    if create_sample:
        sample_size = int(input("Sample size (default 100000): ") or "100000")
        sample_file = "dem_sample.csv"
        sample_large_dataset(csv_path, sample_size, sample_file)
        csv_path = sample_file
    
    try:
        # Load data
        data = load_dem_simple(csv_path)
        
        # Create matrix
        elevation_matrix, x_coords, y_coords = create_matrix_simple(data)
        
        # Save results
        save_matrix_simple(elevation_matrix, x_coords, y_coords)
        
        print("\nConversion complete!")
        print(f"Matrix shape: {elevation_matrix.shape}")
        print(f"Elevation range: {np.nanmin(elevation_matrix):.2f} to {np.nanmax(elevation_matrix):.2f}")
        
        # Print usage example
        print("\nTo load the matrix later:")
        print("import numpy as np")
        print("elevation_matrix = np.load('dem_matrix_elevation.npy')")
        print("x_coords = np.load('dem_matrix_longitude.npy')")
        print("y_coords = np.load('dem_matrix_latitude.npy')")
        
        return elevation_matrix, x_coords, y_coords
        
    except Exception as e:
        print(f"Error: {e}")
        return None, None, None

if __name__ == "__main__":
    elevation_matrix, x_coords, y_coords = main()
