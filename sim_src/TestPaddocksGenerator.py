"""
Paddock Map Generator Script

This script demonstrates creating and saving multiple paddock maps for robot testing.
It creates various test environments with different obstacles and saves them permanently.
"""
import os
import numpy as np
from PaddockMap import PaddockMap  # Assuming the class is saved in paddock_map.py

def create_simple_rectangular_paddock():
    """Create a simple rectangular paddock map"""
    print("\n=== Creating Simple Rectangular Paddock ===")
    
    # Define the four corners of a rectangular paddock
    # Using relative coordinates around a central point for easier testing
    center_lat, center_lon = -33.890, 151.190  # Example coordinates
    width_deg, height_deg = 0.001, 0.0008  # Roughly 100m x 80m
    
    corners = [
        (center_lat - height_deg/2, center_lon - width_deg/2),  # Bottom left
        (center_lat - height_deg/2, center_lon + width_deg/2),  # Bottom right
        (center_lat + height_deg/2, center_lon + width_deg/2),  # Top right
        (center_lat + height_deg/2, center_lon - width_deg/2),  # Top left
    ]
    
    # Create map with 0.5m grid resolution
    paddock = PaddockMap(corners, grid_resolution=0.5, map_folder="saved_maps")
    
    # Add a tree in the middle
    paddock.add_circular_object(center_lat, center_lon, radius_meters=2.0, 
                               object_type="tree", name="Central Tree")
    
    # Add a water trough
    paddock.add_rectangular_object(
        center_lat - 0.0001, center_lon + 0.0003,
        center_lat - 0.00005, center_lon + 0.0004,
        object_type="water_trough", name="Water Trough"
    )
    
    # Save the map
    filename = paddock.save_grid_data("saved_maps/simple_rectangular_paddock.npz")
    
    # Robot starting position (bottom left corner, but inside the paddock)
    robot_start = (center_lat - height_deg/2 + 0.00005, center_lon - width_deg/2 + 0.00005)
    
    return filename, robot_start

def create_complex_paddock():
    """Create a more complex paddock with various obstacles"""
    print("\n=== Creating Complex Paddock ===")
    
    # Define an irregular pentagon paddock
    center_lat, center_lon = -33.885, 151.185
    
    corners = [
        (center_lat - 0.0008, center_lon - 0.0005),
        (center_lat - 0.0005, center_lon + 0.0008),
        (center_lat + 0.0007, center_lon + 0.0007),
        (center_lat + 0.0009, center_lon + 0.0000),
        (center_lat + 0.0002, center_lon - 0.0009),
    ]
    
    # Create map with 0.25m grid resolution (higher resolution)
    paddock = PaddockMap(corners, grid_resolution=0.25, map_folder="saved_maps")
    
    # Add several trees
    paddock.add_circular_object(center_lat + 0.0002, center_lon - 0.0002, radius_meters=1.5, 
                               object_type="tree", name="Tree 1")
    paddock.add_circular_object(center_lat - 0.0003, center_lon + 0.0004, radius_meters=2.0, 
                               object_type="tree", name="Tree 2")
    paddock.add_circular_object(center_lat + 0.0005, center_lon + 0.0003, radius_meters=1.8, 
                               object_type="tree", name="Tree 3")
    
    # Add a water trough
    paddock.add_rectangular_object(
        center_lat - 0.0001, center_lon - 0.0003,
        center_lat + 0.0001, center_lon - 0.0001,
        object_type="water_trough", name="Water Trough"
    )
    
    # Add some fence posts along one side
    for i in range(5):
        factor = i / 4.0  # Goes from 0 to 1
        lat = corners[0][0] * (1-factor) + corners[1][0] * factor
        lon = corners[0][1] * (1-factor) + corners[1][1] * factor
        paddock.add_point_object(lat, lon, object_type="fence_post", name=f"Post {i+1}")
    
    # Add a rocky area
    paddock.add_circular_object(center_lat - 0.0002, center_lon - 0.0005, radius_meters=3.0, 
                               object_type="rocky_area", name="Rocky Zone")
    
    # Save the map
    filename = paddock.save_grid_data("saved_maps/complex_paddock.npz")
    
    # Robot starting position (near the center but avoiding obstacles)
    robot_start = (center_lat, center_lon - 0.0004)
    
    return filename, robot_start

def create_obstacle_course():
    """Create a paddock designed as an obstacle course for testing navigation"""
    print("\n=== Creating Obstacle Course Paddock ===")
    
    # Define a rectangular paddock
    center_lat, center_lon = -33.895, 151.195
    width_deg, height_deg = 0.0012, 0.0010
    
    corners = [
        (center_lat - height_deg/2, center_lon - width_deg/2),
        (center_lat - height_deg/2, center_lon + width_deg/2),
        (center_lat + height_deg/2, center_lon + width_deg/2),
        (center_lat + height_deg/2, center_lon - width_deg/2),
    ]
    
    # Create map with 0.2m grid resolution (high resolution)
    paddock = PaddockMap(corners, grid_resolution=0.2, map_folder="saved_maps")
    
    # Create a "maze" of obstacles
    # Central barrier
    paddock.add_rectangular_object(
        center_lat - 0.0003, center_lon - 0.0003,
        center_lat + 0.0003, center_lon - 0.0002,
        object_type="barrier", name="Central Barrier"
    )
    
    # Side barriers
    paddock.add_rectangular_object(
        center_lat - 0.0003, center_lon - 0.0001,
        center_lat - 0.0001, center_lon + 0.0004,
        object_type="barrier", name="Left Barrier"
    )
    
    paddock.add_rectangular_object(
        center_lat + 0.0001, center_lon + 0.0002,
        center_lat + 0.0003, center_lon + 0.0005,
        object_type="barrier", name="Right Barrier"
    )
    
    # Add some circular obstacles
    paddock.add_circular_object(center_lat - 0.0002, center_lon + 0.0002, radius_meters=1.0, 
                               object_type="obstacle", name="Obstacle 1")
    paddock.add_circular_object(center_lat + 0.0004, center_lon - 0.0001, radius_meters=1.2, 
                               object_type="obstacle", name="Obstacle 2")
    paddock.add_circular_object(center_lat - 0.0004, center_lon - 0.0004, radius_meters=0.8, 
                               object_type="obstacle", name="Obstacle 3")
    
    # Add a goal point at the far end
    paddock.add_point_object(center_lat + 0.0004, center_lon + 0.0004, 
                            object_type="goal", name="Goal Point")
    
    # Save the map
    filename = paddock.save_grid_data("saved_maps/obstacle_course.npz")
    
    # Robot starting position (bottom left corner inside the paddock)
    robot_start = (center_lat - height_deg/2 + 0.00008, center_lon - width_deg/2 + 0.00008)
    
    return filename, robot_start

def main():
    """Main function to create and save multiple paddock maps"""
    # Create directory for saved maps if it doesn't exist
    if not os.path.exists("saved_maps"):
        os.makedirs("saved_maps")
        print("Created directory: saved_maps")
    
    # Create and save three different maps
    map_data = []
    
    filename1, start_pos1 = create_simple_rectangular_paddock()
    map_data.append(("Simple Rectangular Paddock", filename1, start_pos1))
    
    filename2, start_pos2 = create_complex_paddock()
    map_data.append(("Complex Paddock", filename2, start_pos2))
    
    filename3, start_pos3 = create_obstacle_course()
    map_data.append(("Obstacle Course", filename3, start_pos3))
    
    # Print summary of created maps
    print("\n=== MAP GENERATION SUMMARY ===")
    print("The following maps have been created and saved:")
    
    for name, filename, start_pos in map_data:
        print(f"\n{name}")
        print(f"  - File: {filename}")
        print(f"  - Robot starting position (lat, lon): {start_pos[0]:.6f}, {start_pos[1]:.6f}")
    
    print("\n=== HOW TO LOAD MAPS ===")
    print("You can load any of these maps using:")
    print("    paddock = PaddockMap.load_grid_data('filename.npz')")
    print("\nThen place your robot at the suggested starting position.")

if __name__ == "__main__":
    main()