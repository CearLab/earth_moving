#!/usr/bin/env python3
"""
Example script showing how to work with the converted DEM matrix.
"""

import numpy as np
import matplotlib.pyplot as plt

def load_dem_matrix():
    """Load the DEM matrix and coordinates."""
    print("Loading DEM matrix...")
    
    elevation_matrix = np.load('dem_matrix_elevation.npy')
    longitude_coords = np.load('dem_matrix_longitude.npy')
    latitude_coords = np.load('dem_matrix_latitude.npy')
    
    print(f"Matrix shape: {elevation_matrix.shape}")
    print(f"Longitude range: {longitude_coords.min():.8f} to {longitude_coords.max():.8f}")
    print(f"Latitude range: {latitude_coords.min():.8f} to {latitude_coords.max():.8f}")
    print(f"Elevation range: {np.nanmin(elevation_matrix):.2f} to {np.nanmax(elevation_matrix):.2f}")
    
    return elevation_matrix, longitude_coords, latitude_coords

def visualize_matrix(elevation_matrix, longitude_coords, latitude_coords):
    """Create basic visualizations of the DEM matrix."""
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # 1. Elevation map
    extent = [longitude_coords.min(), longitude_coords.max(), 
              latitude_coords.min(), latitude_coords.max()]
    
    im1 = axes[0,0].imshow(elevation_matrix, extent=extent, origin='lower', 
                          cmap='terrain', aspect='auto')
    axes[0,0].set_title('DEM Elevation Map')
    axes[0,0].set_xlabel('Longitude')
    axes[0,0].set_ylabel('Latitude')
    plt.colorbar(im1, ax=axes[0,0], label='Elevation (m)')
    
    # 2. Histogram of elevations
    valid_elevations = elevation_matrix[~np.isnan(elevation_matrix)]
    axes[0,1].hist(valid_elevations, bins=50, alpha=0.7, color='green')
    axes[0,1].set_title('Elevation Distribution')
    axes[0,1].set_xlabel('Elevation (m)')
    axes[0,1].set_ylabel('Frequency')
    axes[0,1].grid(True, alpha=0.3)
    
    # 3. Data coverage (where we have elevation data)
    coverage = ~np.isnan(elevation_matrix)
    axes[1,0].imshow(coverage, extent=extent, origin='lower', cmap='gray', aspect='auto')
    axes[1,0].set_title('Data Coverage (White = Data)')
    axes[1,0].set_xlabel('Longitude')
    axes[1,0].set_ylabel('Latitude')
    
    # 4. Elevation profile through center
    center_row = elevation_matrix.shape[0] // 2
    profile = elevation_matrix[center_row, :]
    valid_indices = ~np.isnan(profile)
    
    if np.any(valid_indices):
        axes[1,1].plot(longitude_coords[valid_indices], profile[valid_indices])
        axes[1,1].set_title('Elevation Profile (Center Row)')
        axes[1,1].set_xlabel('Longitude')
        axes[1,1].set_ylabel('Elevation (m)')
        axes[1,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('dem_visualization.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print("Visualization saved as 'dem_visualization.png'")

def matrix_operations_examples(elevation_matrix, longitude_coords, latitude_coords):
    """Show examples of common operations with the elevation matrix."""
    
    print("\\nMatrix Operations Examples:")
    print("==========================")
    
    # 1. Get elevation at specific coordinates
    def get_elevation_at(lon, lat):
        lon_idx = np.argmin(np.abs(longitude_coords - lon))
        lat_idx = np.argmin(np.abs(latitude_coords - lat))
        return elevation_matrix[lat_idx, lon_idx]
    
    # Test with middle coordinates
    mid_lon = longitude_coords[len(longitude_coords)//2]
    mid_lat = latitude_coords[len(latitude_coords)//2]
    elevation = get_elevation_at(mid_lon, mid_lat)
    
    print(f"Elevation at center ({mid_lon:.6f}, {mid_lat:.6f}): {elevation:.2f} m")
    
    # 2. Find highest and lowest points
    valid_mask = ~np.isnan(elevation_matrix)
    if np.any(valid_mask):
        max_idx = np.unravel_index(np.nanargmax(elevation_matrix), elevation_matrix.shape)
        min_idx = np.unravel_index(np.nanargmin(elevation_matrix), elevation_matrix.shape)
        
        max_elevation = elevation_matrix[max_idx]
        min_elevation = elevation_matrix[min_idx]
        max_coords = (longitude_coords[max_idx[1]], latitude_coords[max_idx[0]])
        min_coords = (longitude_coords[min_idx[1]], latitude_coords[min_idx[0]])
        
        print(f"Highest point: {max_elevation:.2f} m at {max_coords}")
        print(f"Lowest point: {min_elevation:.2f} m at {min_coords}")
    
    # 3. Calculate basic statistics
    valid_elevations = elevation_matrix[~np.isnan(elevation_matrix)]
    if len(valid_elevations) > 0:
        print(f"\\nElevation Statistics:")
        print(f"Mean: {np.mean(valid_elevations):.2f} m")
        print(f"Std: {np.std(valid_elevations):.2f} m")
        print(f"Valid points: {len(valid_elevations):,} / {elevation_matrix.size:,}")
        print(f"Coverage: {len(valid_elevations)/elevation_matrix.size*100:.1f}%")
    
    # 4. Extract a smaller region
    print(f"\\nExtracting subregion...")
    rows, cols = elevation_matrix.shape
    start_row, end_row = rows//4, 3*rows//4
    start_col, end_col = cols//4, 3*cols//4
    
    subregion = elevation_matrix[start_row:end_row, start_col:end_col]
    sub_lons = longitude_coords[start_col:end_col]
    sub_lats = latitude_coords[start_row:end_row]
    
    print(f"Subregion shape: {subregion.shape}")
    print(f"Subregion bounds: {sub_lons.min():.6f} to {sub_lons.max():.6f} lon, "
          f"{sub_lats.min():.6f} to {sub_lats.max():.6f} lat")
    
    return subregion, sub_lons, sub_lats

def save_as_different_formats(elevation_matrix, longitude_coords, latitude_coords):
    """Show how to save the matrix in different formats."""
    
    print("\\nSaving in different formats...")
    
    # 1. Save as CSV (only if not too large)
    if elevation_matrix.size < 1000000:  # Less than 1M elements
        np.savetxt('elevation_matrix.csv', elevation_matrix, delimiter=',')
        print("Saved as CSV: elevation_matrix.csv")
    else:
        print("Matrix too large for CSV format")
    
    # 2. Save coordinates as CSV
    np.savetxt('longitude_coords.csv', longitude_coords, delimiter=',')
    np.savetxt('latitude_coords.csv', latitude_coords, delimiter=',')
    print("Coordinates saved as CSV files")
    
    # 3. Save a subset for testing
    subset = elevation_matrix[::10, ::10]  # Every 10th point
    np.savetxt('elevation_matrix_subset.csv', subset, delimiter=',')
    print(f"Subset saved as CSV: {subset.shape}")

def main():
    """Main function demonstrating matrix usage."""
    
    # Load the matrix
    elevation_matrix, longitude_coords, latitude_coords = load_dem_matrix()
    
    # Create visualizations
    visualize_matrix(elevation_matrix, longitude_coords, latitude_coords)
    
    # Show matrix operations
    subregion, sub_lons, sub_lats = matrix_operations_examples(
        elevation_matrix, longitude_coords, latitude_coords)
    
    # Save in different formats
    save_as_different_formats(elevation_matrix, longitude_coords, latitude_coords)
    
    print("\\n" + "="*50)
    print("SUCCESS! Your DEM is now a 2D matrix!")
    print("="*50)
    print(f"Matrix shape: {elevation_matrix.shape}")
    print(f"Access elevation at row i, col j: elevation_matrix[i, j]")
    print(f"Longitude at column j: longitude_coords[j]")
    print(f"Latitude at row i: latitude_coords[i]")
    print("\\nThe matrix is ready for any spatial analysis!")

if __name__ == "__main__":
    main()
