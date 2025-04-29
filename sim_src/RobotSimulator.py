import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time
from PaddockMap import PaddockMap

class RobotSimulator:
    def __init__(self, paddock_map, start_pos, speed=0.5):
        """
        Initialize robot simulator
        
        Args:
            paddock_map: PaddockMap object
            start_pos: (lat, lon) tuple for starting position
            speed: Robot speed in meters per second
        """
        self.paddock = paddock_map
        self.position = start_pos
        self.speed = speed
        self.path = [start_pos]
        self.grid_path = [self.paddock.gps_to_grid(start_pos[0], start_pos[1])]
        
        # Check if starting position is valid
        if not self.paddock.is_inside_boundary(start_pos[0], start_pos[1]):
            print("WARNING: Starting position is outside the paddock boundary!")
    
    def move_to(self, target_lat, target_lon, plot=False, save_dir=None):
        """
        Move robot to target position, avoiding obstacles
        This is a very simple direct path with no actual pathfinding
        
        Args:
            target_lat, target_lon: Target GPS coordinates
            plot: Whether to plot the movement in real-time
            save_dir: Directory to save images instead of displaying them
        """
        print(f"Moving from ({self.position[0]:.6f}, {self.position[1]:.6f}) "
              f"to ({target_lat:.6f}, {target_lon:.6f})")
        
        # Convert to grid coordinates
        start_grid = self.paddock.gps_to_grid(self.position[0], self.position[1])
        target_grid = self.paddock.gps_to_grid(target_lat, target_lon)
        
        # Check if target is inside boundary
        if not self.paddock.is_inside_boundary(target_lat, target_lon):
            print("ERROR: Target position is outside the paddock boundary!")
            return False
        
        # Just go in a straight line (very naive approach - no real path planning)
        # In a real robot, you'd want a proper path planning algorithm here
        
        # Calculate path using Bresenham's algorithm
        path_grid = self._bresenham_line(start_grid[0], start_grid[1], 
                                         target_grid[0], target_grid[1])
        
        # Convert back to GPS coordinates
        path_gps = [self.paddock.grid_to_gps(x, y) for x, y in path_grid]
        
        # Initialize plot if needed
        if plot:
            fig, ax = self._init_plot()
        
        # Move along the path
        obstacle_hit = False
        
        # Determine how often to save frames (if saving)
        save_interval = max(1, len(path_grid) // 10) if save_dir else 0
        step_counter = 0  # For unique filenames
        
        for i, (grid_pos, gps_pos) in enumerate(zip(path_grid, path_gps)):
            x, y = grid_pos
            
            # Check for obstacles
            if 0 <= y < self.paddock.grid_height and 0 <= x < self.paddock.grid_width:
                cell_value = self.paddock.grid[y, x]
                if cell_value > 0:  # This is an obstacle or boundary
                    print(f"Obstacle detected at step {i}!")
                    obstacle_hit = True
                    break
            
            # Update position
            self.position = gps_pos
            self.path.append(gps_pos)
            self.grid_path.append(grid_pos)
            
            # Update plot
            if plot:
                # If saving and it's a save interval or the last point
                if save_dir and (i % save_interval == 0 or i == len(path_grid) - 1):
                    save_path = os.path.join(save_dir, f"step_{step_counter:04d}.png")
                    self._update_plot(ax, save_path)
                    step_counter += 1
                else:
                    self._update_plot(ax)
                    # Only pause if we're displaying, not saving
                    if not save_dir:
                        plt.pause(0.1)  # Small delay to see movement
        
        if obstacle_hit:
            print("Movement stopped due to obstacle in path!")
            return False
        
        print(f"Arrived at destination: ({self.position[0]:.6f}, {self.position[1]:.6f})")
        return True
    
    def scan_surroundings(self, radius_meters=5.0):
        """
        Scan surroundings for obstacles within given radius
        
        Args:
            radius_meters: Scan radius in meters
        
        Returns:
            List of detected objects
        """
        # Convert radius to grid cells
        radius_grid = int(radius_meters / self.paddock.grid_resolution)
        
        # Get current position in grid coordinates
        current_grid = self.paddock.gps_to_grid(self.position[0], self.position[1])
        x_center, y_center = current_grid
        
        detected_cells = {}  # Value -> list of positions
        
        # Scan in a circle
        for y in range(y_center - radius_grid, y_center + radius_grid + 1):
            for x in range(x_center - radius_grid, x_center + radius_grid + 1):
                # Check if within circle and within grid bounds
                if ((x - x_center)**2 + (y - y_center)**2 <= radius_grid**2 and
                    0 <= y < self.paddock.grid_height and 0 <= x < self.paddock.grid_width):
                    cell_value = self.paddock.grid[y, x]
                    if cell_value > 0:  # This is something (not free space)
                        if cell_value not in detected_cells:
                            detected_cells[cell_value] = []
                        detected_cells[cell_value].append((x, y))
        
        # Convert to more readable format
        detections = []
        for value, positions in detected_cells.items():
            # Find object type from value
            obj_type = "unknown"
            for type_name, obj_info in self.paddock.objects.items():
                if obj_info["value"] == value:
                    obj_type = type_name
                    break
            
            # Convert centroid to GPS
            if positions:
                avg_x = sum(p[0] for p in positions) / len(positions)
                avg_y = sum(p[1] for p in positions) / len(positions)
                gps_pos = self.paddock.grid_to_gps(int(avg_x), int(avg_y))
                
                detections.append({
                    "type": obj_type,
                    "position": gps_pos,
                    "grid_cells": len(positions)
                })
        
        # Print results
        print(f"\nScan results (radius: {radius_meters}m):")
        for i, detection in enumerate(detections):
            print(f"  {i+1}. {detection['type']} at "
                  f"({detection['position'][0]:.6f}, {detection['position'][1]:.6f}), "
                  f"size: {detection['grid_cells']} cells")
        
        return detections
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Generate points in a line using Bresenham's algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                if x0 == x1:
                    break
                err -= dy
                x0 += sx
            if e2 < dx:
                if y0 == y1:
                    break
                err += dx
                y0 += sy
                
        return points
    
    def _init_plot(self):
        """Initialize plot for visualization"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Create colormap with different colors for different object types
        num_objects = max([obj_info["value"] for obj_info in self.paddock.objects.values()]) + 1
        colors = ['white'] + ['C{}'.format(i % 10) for i in range(num_objects)]
        cmap = ListedColormap(colors)
        
        # Plot the grid
        ax.imshow(self.paddock.grid, cmap=cmap, origin='upper')
        
        # Add legend
        legend_elements = []
        for obj_type, obj_info in self.paddock.objects.items():
            color = obj_info["color"]
            if color.startswith('C'):
                color = f'tab:{["blue", "orange", "green", "red", "purple", "brown", "pink", "gray", "olive", "cyan"][int(color[1:])]}'
            
            # Add to legend
            legend_elements.append(plt.Rectangle((0, 0), 1, 1, color=color, label=obj_type))
        
        # Add robot path
        ax.plot([p[0] for p in self.grid_path], 
                [p[1] for p in self.grid_path], 
                'b-', linewidth=1.5, label='Robot Path')
        
        # Add robot position - using a list for x and y to ensure they're sequences
        current_pos = self.grid_path[-1]
        ax.plot([current_pos[0]], [current_pos[1]], 'ro', markersize=8, label='Robot')
        
        # Add legend
        legend_elements.append(plt.Line2D([0], [0], color='b', lw=1.5, label='Path'))
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', 
                                         markerfacecolor='r', markersize=8, label='Robot'))
        ax.legend(handles=legend_elements, loc='upper right')
        
        ax.set_title('Paddock Map and Robot Path')
        
        return fig, ax
    
    def _update_plot(self, ax, save_path=None):
        """Update plot with current position and optionally save to file
        
        Args:
            ax: Matplotlib axis object
            save_path: If provided, save the plot to this path instead of displaying
        """
        # Update path line
        ax.lines[0].set_data([p[0] for p in self.grid_path], [p[1] for p in self.grid_path])
        
        # Update robot position - using lists to ensure they're sequences
        current_pos = self.grid_path[-1]
        ax.lines[1].set_data([current_pos[0]], [current_pos[1]])
        
        # Save if path provided, otherwise just draw
        if save_path:
            plt.savefig(save_path)
        else:
            plt.draw()


def load_paddock_map(filename):
    """Load a paddock map from file"""
    if not os.path.exists(filename):
        print(f"Error: Map file {filename} not found!")
        return None
    
    try:
        paddock = PaddockMap.load_grid_data(filename)
        print(f"Successfully loaded map: {filename}")
        return paddock
    except Exception as e:
        print(f"Error loading map: {str(e)}")
        return None


def test_robot_navigation(map_filename, start_pos):
    """Test robot navigation on a given map"""
    # Load the map
    paddock = load_paddock_map(map_filename)
    if paddock is None:
        return
    
    # Create robot simulator
    robot = RobotSimulator(paddock, start_pos)
    
    # Scan surroundings
    robot.scan_surroundings(radius_meters=10.0)
    
    # Plot the initial state
    fig, ax = robot._init_plot()
    plt.pause(1)  # Give time to display
    
    # Navigate to a few points
    # For testing, we'll just go to a few points in the paddock
    # Convert to relative offsets from the center to ensure we stay in bounds
    center_lat = (paddock.min_lat + paddock.max_lat) / 2
    center_lon = (paddock.min_lon + paddock.max_lon) / 2
    height_span = paddock.max_lat - paddock.min_lat
    width_span = paddock.max_lon - paddock.min_lon
    
    # Define waypoints as offsets from center
    waypoints = [
        (center_lat + 0.2 * height_span, center_lon - 0.2 * width_span),
        (center_lat + 0.2 * height_span, center_lon + 0.2 * width_span),
        (center_lat - 0.2 * height_span, center_lon + 0.2 * width_span),
        (center_lat - 0.2 * height_span, center_lon - 0.2 * width_span),
        (center_lat, center_lon)  # Return to center
    ]
    
    # Visit each waypoint
    for i, waypoint in enumerate(waypoints):
        print(f"\nMoving to waypoint {i+1}/{len(waypoints)}...")
        success = robot.move_to(waypoint[0], waypoint[1], plot=True,save_dir="robot_maps")
        
        if success:
            robot.scan_surroundings(radius_meters=5.0)
            plt.pause(1)  # Pause to see the result
        else:
            print("Navigation failed at waypoint", i+1)
            break
    
    print("\nNavigation test completed!")
    plt.show()  # Keep plot open