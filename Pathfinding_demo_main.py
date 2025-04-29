import numpy as np
import matplotlib.pyplot as plt
from math import radians, sin, cos, sqrt, atan2, degrees, pi
import os
from datetime import datetime
from matplotlib.patches import Circle
import time


def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS points in meters"""
    R = 6371000  # Earth radius in meters
    
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2 - lat1)
    delta_lambda = radians(lon2 - lon1)
    
    a = sin(delta_phi/2)**2 + cos(phi1) * cos(phi2) * sin(delta_lambda/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    return R * c

class PaddockMap:
    def __init__(self, corner_coords, grid_resolution=1.0, map_folder="robot_maps"):
        """
        Initialize paddock map
        
        Args:
            corner_coords: List of (lat, lon) tuples defining paddock corners
            grid_resolution: Grid cell size in meters
            map_folder: Folder to save map images
        """
        self.corner_coords = corner_coords
        self.grid_resolution = grid_resolution
        self.map_folder = map_folder
        
        # Cell values:
        # 0 = free space
        # 1 = boundary
        # 2 = permanent object
        # 3+ = can be used for different object types
        
        # Create map folder if it doesn't exist
        if not os.path.exists(map_folder):
            os.makedirs(map_folder)
            print(f"Created map folder: {map_folder}")
        
        # Find min/max lat/lon to create bounding box
        lats = [coord[0] for coord in corner_coords]
        lons = [coord[1] for coord in corner_coords]
        self.min_lat, self.max_lat = min(lats), max(lats)
        self.min_lon, self.max_lon = min(lons), max(lons)
        
        # Calculate paddock dimensions in meters
        width_meters = haversine_distance(self.min_lat, self.min_lon, 
                                          self.min_lat, self.max_lon)
        height_meters = haversine_distance(self.min_lat, self.min_lon, 
                                           self.max_lat, self.min_lon)
        
        # Create grid dimensions
        self.grid_width = int(width_meters / grid_resolution) + 1
        self.grid_height = int(height_meters / grid_resolution) + 1
        
        # Initialize grid (0 = free space)
        self.grid = np.zeros((self.grid_height, self.grid_width))
        
        # Dictionary to track objects in the paddock
        self.objects = {
            "boundary": {"color": "black", "value": 1, "items": []},
            "permanent_object": {"color": "red", "value": 2, "items": []}
        }
        
        # Mark boundaries on grid
        self._mark_boundaries()
    
    def _mark_boundaries(self):
        """Mark paddock boundaries on the grid"""
        boundary_points = []
        
        # For each pair of adjacent corners, draw a line
        for i in range(len(self.corner_coords)):
            start = self.corner_coords[i]
            end = self.corner_coords[(i + 1) % len(self.corner_coords)]
            
            # Add to boundary object list
            boundary_points.append({
                "type": "line",
                "start": start,
                "end": end
            })
            
            # Convert corner points to grid coordinates
            start_grid = self.gps_to_grid(start[0], start[1])
            end_grid = self.gps_to_grid(end[0], end[1])
            
            # Draw line between points using Bresenham's algorithm
            x0, y0 = start_grid
            x1, y1 = end_grid
            
            # Simple implementation of Bresenham's
            is_steep = abs(y1 - y0) > abs(x1 - x0)
            if is_steep:
                x0, y0 = y0, x0
                x1, y1 = y1, x1
            
            if x0 > x1:
                x0, x1 = x1, x0
                y0, y1 = y1, y0
            
            dx = x1 - x0
            dy = abs(y1 - y0)
            error = dx // 2
            y = y0
            
            if y0 < y1:
                y_step = 1
            else:
                y_step = -1
            
            for x in range(x0, x1 + 1):
                if is_steep:
                    if 0 <= y < self.grid_width and 0 <= x < self.grid_height:
                        self.grid[x, y] = 1
                else:
                    if 0 <= y < self.grid_height and 0 <= x < self.grid_width:
                        self.grid[y, x] = 1
                
                error -= dy
                if error < 0:
                    y += y_step
                    error += dx
        
        # Store boundary in objects dictionary
        self.objects["boundary"]["items"] = boundary_points
    
    def add_circular_object(self, lat, lon, radius_meters, object_type="permanent_object", name=None):
        """
        Add a circular object to the map
        
        Args:
            lat, lon: GPS coordinates of object center
            radius_meters: Radius of object in meters
            object_type: Type of object (default: permanent_object)
            name: Optional name for the object
        """
        # Create a new object type if it doesn't exist
        if object_type not in self.objects:
            next_value = max([obj_info["value"] for obj_info in self.objects.values()]) + 1
            self.objects[object_type] = {
                "color": f"C{next_value-1}",  # Use matplotlib color cycle
                "value": next_value,
                "items": []
            }
        
        # Add object to the list
        object_info = {
            "type": "circle",
            "center": (lat, lon),
            "radius": radius_meters,
            "name": name
        }
        self.objects[object_type]["items"].append(object_info)
        
        # Mark on grid
        center_grid = self.gps_to_grid(lat, lon)
        radius_grid = int(radius_meters / self.grid_resolution)
        value = self.objects[object_type]["value"]
        
        # Draw circle using midpoint circle algorithm
        self._draw_circle_on_grid(center_grid[0], center_grid[1], radius_grid, value)
        
        print(f"Added {object_type} at ({lat:.6f}, {lon:.6f}) with radius {radius_meters}m")
    
    def add_rectangular_object(self, lat1, lon1, lat2, lon2, object_type="permanent_object", name=None):
        """
        Add a rectangular object to the map defined by opposite corners
        
        Args:
            lat1, lon1: GPS coordinates of first corner
            lat2, lon2: GPS coordinates of opposite corner
            object_type: Type of object (default: permanent_object)
            name: Optional name for the object
        """
        # Create a new object type if it doesn't exist
        if object_type not in self.objects:
            next_value = max([obj_info["value"] for obj_info in self.objects.values()]) + 1
            self.objects[object_type] = {
                "color": f"C{next_value-1}",  # Use matplotlib color cycle
                "value": next_value,
                "items": []
            }
        
        # Add object to the list
        object_info = {
            "type": "rectangle",
            "corner1": (lat1, lon1),
            "corner2": (lat2, lon2),
            "name": name
        }
        self.objects[object_type]["items"].append(object_info)
        
        # Mark on grid
        grid1 = self.gps_to_grid(lat1, lon1)
        grid2 = self.gps_to_grid(lat2, lon2)
        value = self.objects[object_type]["value"]
        
        # Ensure grid1 has the smaller coordinates
        x1, y1 = min(grid1[0], grid2[0]), min(grid1[1], grid2[1])
        x2, y2 = max(grid1[0], grid2[0]), max(grid1[1], grid2[1])
        
        # Draw rectangle on grid
        for x in range(x1, x2 + 1):
            for y in range(y1, y2 + 1):
                if 0 <= y < self.grid_height and 0 <= x < self.grid_width:
                    self.grid[y, x] = value
        
        print(f"Added {object_type} rectangle from ({lat1:.6f}, {lon1:.6f}) to ({lat2:.6f}, {lon2:.6f})")
    
    def add_point_object(self, lat, lon, object_type="permanent_object", name=None):
        """
        Add a point object to the map
        
        Args:
            lat, lon: GPS coordinates of object
            object_type: Type of object (default: permanent_object)
            name: Optional name for the object
        """
        # Create a new object type if it doesn't exist
        if object_type not in self.objects:
            next_value = max([obj_info["value"] for obj_info in self.objects.values()]) + 1
            self.objects[object_type] = {
                "color": f"C{next_value-1}",  # Use matplotlib color cycle
                "value": next_value,
                "items": []
            }
        
        # Add object to the list
        object_info = {
            "type": "point",
            "location": (lat, lon),
            "name": name
        }
        self.objects[object_type]["items"].append(object_info)
        
        # Mark on grid
        grid_x, grid_y = self.gps_to_grid(lat, lon)
        value = self.objects[object_type]["value"]
        
        if 0 <= grid_y < self.grid_height and 0 <= grid_x < self.grid_width:
            self.grid[grid_y, grid_x] = value
        
        print(f"Added {object_type} point at ({lat:.6f}, {lon:.6f})")
    
    def _draw_circle_on_grid(self, center_x, center_y, radius, value):
        """Draw a circle on the grid using midpoint circle algorithm"""
        # Set points in a circle
        for y in range(-radius, radius + 1):
            for x in range(-radius, radius + 1):
                if x*x + y*y <= radius*radius:
                    grid_x, grid_y = center_x + x, center_y + y
                    if 0 <= grid_y < self.grid_height and 0 <= grid_x < self.grid_width:
                        self.grid[grid_y, grid_x] = value
    
    def gps_to_grid(self, lat, lon):
        """Convert GPS coordinates to grid coordinates"""
        # Calculate distance from origin (min_lat, min_lon)
        x_meters = haversine_distance(self.min_lat, self.min_lon, 
                                      self.min_lat, lon)
        y_meters = haversine_distance(self.min_lat, self.min_lon, 
                                      lat, self.min_lon)
        
        # Convert to grid cells
        x_grid = int(x_meters / self.grid_resolution)
        y_grid = int(y_meters / self.grid_resolution)
        
        # Invert y-axis (grid y increases downward)
        y_grid = self.grid_height - y_grid - 1
        
        return (x_grid, y_grid)
    
    def grid_to_gps(self, x_grid, y_grid):
        """Convert grid coordinates to GPS coordinates (approximate)"""
        # Invert y-axis again
        y_grid = self.grid_height - y_grid - 1
        
        # Calculate meters from origin
        x_meters = x_grid * self.grid_resolution
        y_meters = y_grid * self.grid_resolution
        
        # Approximate conversion back to GPS
        # This is a simplification and will have some error
        # For better accuracy, use proper projection libraries
        earth_radius = 6371000  # meters
        
        lon = self.min_lon + (x_meters / (earth_radius * cos(radians(self.min_lat)))) * (180 / np.pi)
        lat = self.min_lat + (y_meters / earth_radius) * (180 / np.pi)
        
        return (lat, lon)
    
    def is_inside_boundary(self, lat, lon):
        """Check if a GPS position is inside the paddock boundary"""
        x, y = self.gps_to_grid(lat, lon)
        
        # Simple flood fill check - this requires the boundary to be closed
        # A more robust solution would use ray casting or point-in-polygon algorithms
        temp_grid = self.grid.copy()
        
        # Use flood fill from edges to mark outside areas
        queue = [(0, 0)]
        while queue:
            cx, cy = queue.pop(0)
            if cx < 0 or cy < 0 or cx >= self.grid_width or cy >= self.grid_height:
                continue
            if temp_grid[cy, cx] != 0:
                continue
            
            temp_grid[cy, cx] = -1  # Mark as outside
            queue.append((cx+1, cy))
            queue.append((cx-1, cy))
            queue.append((cx, cy+1))
            queue.append((cx, cy-1))
        
        # Check if our point is marked as outside
        if 0 <= y < self.grid_height and 0 <= x < self.grid_width:
            return temp_grid[y, x] != -1
        return False
    
    def plot_map(self, robot_position=None, target_position=None, path=None, save=True, show=True, show_legend=True):
        """
        Plot the paddock map with optional robot and target positions
        
        Args:
            robot_position: (lat, lon) tuple of robot position
            target_position: (lat, lon) tuple of target position
            path: List of (lat, lon) tuples representing a path
            save: Whether to save the map to file
            show: Whether to display the map
            show_legend: Whether to show the legend
        """
        plt.figure(figsize=(12, 10))
        
        # Create a custom colormap for different object types
        from matplotlib.colors import ListedColormap
        num_objects = len(self.objects)
        cmap_colors = ['white']  # 0 = free space
        
        # Add colors for each object type
        for obj_type, obj_info in sorted(self.objects.items(), key=lambda x: x[1]["value"]):
            cmap_colors.append(obj_info["color"])
        
        # Create the colormap
        cmap = ListedColormap(cmap_colors)
        
        # Plot the grid
        plt.imshow(self.grid, cmap=cmap, origin='lower', 
                   vmin=0, vmax=len(cmap_colors)-1)
        
        # Create legend handles
        legend_handles = []
        for obj_type, obj_info in self.objects.items():
            from matplotlib.patches import Patch
            legend_handles.append(Patch(color=obj_info["color"], label=obj_type.replace("_", " ").title()))
        
        # Plot robot position
        if robot_position:
            robot_x, robot_y = self.gps_to_grid(robot_position[0], robot_position[1])
            robot_marker = plt.plot(robot_x, robot_y, 'o', markersize=10, 
                                    color='blue', markeredgecolor='black', 
                                    markeredgewidth=1.5, label='Robot')
            legend_handles.append(robot_marker[0])
        
        # Plot target position
        if target_position:
            target_x, target_y = self.gps_to_grid(target_position[0], target_position[1])
            target_marker = plt.plot(target_x, target_y, '*', markersize=12, 
                                    color='green', markeredgecolor='black', 
                                    markeredgewidth=1.5, label='Target')
            legend_handles.append(target_marker[0])
        
        # Plot path if provided
        if path:
            path_x = []
            path_y = []
            for point in path:
                x, y = self.gps_to_grid(point[0], point[1])
                path_x.append(x)
                path_y.append(y)
            path_line = plt.plot(path_x, path_y, '-', linewidth=2, 
                                color='blue', label='Path')
            legend_handles.append(path_line[0])
        
        # Add labels for named objects
        for obj_type, obj_info in self.objects.items():
            for item in obj_info["items"]:
                if item.get("name"):
                    if item["type"] == "circle":
                        x, y = self.gps_to_grid(*item["center"])
                    elif item["type"] == "rectangle":
                        x1, y1 = self.gps_to_grid(*item["corner1"])
                        x2, y2 = self.gps_to_grid(*item["corner2"])
                        x, y = (x1 + x2) // 2, (y1 + y2) // 2
                    elif item["type"] == "point":
                        x, y = self.gps_to_grid(*item["location"])
                    else:
                        continue
                    
                    plt.text(x, y, item["name"], fontsize=9, ha='center', 
                            va='center', color='white', fontweight='bold',
                            bbox=dict(facecolor='black', alpha=0.7, pad=1))
        
        plt.title('Paddock Map with Objects')
        plt.grid(True, alpha=0.3)
        
        if show_legend:
            plt.legend(handles=legend_handles, loc='upper right')
        
        # Add scale bar (approximating meters to pixels)
        scale_length_meters = 10  # 10 meter scale bar
        scale_length_pixels = scale_length_meters / self.grid_resolution
        
        # Position at bottom right
        scale_y = self.grid_height * 0.05
        scale_x_start = self.grid_width * 0.75
        scale_x_end = scale_x_start + scale_length_pixels
        
        plt.plot([scale_x_start, scale_x_end], [scale_y, scale_y], 'k-', linewidth=3)
        plt.text((scale_x_start + scale_x_end)/2, scale_y * 1.5, 
                f'{scale_length_meters}m', ha='center', fontsize=9)
        
        # Save the map
        if save:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.map_folder}/paddock_map_{timestamp}.png"
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Map saved to {filename}")
        
        # Show the map
        if show:
            plt.show()
        else:
            plt.close()
    
    def save_grid_data(self, filename=None):
        """Save the grid data to a NumPy file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.map_folder}/paddock_grid_{timestamp}.npz"
        
        # Convert objects dictionary to a format that can be saved
        objects_simplified = {}
        for obj_type, obj_info in self.objects.items():
            objects_simplified[obj_type] = {
                "color": obj_info["color"],
                "value": obj_info["value"],
                "items_count": len(obj_info["items"])
            }
            
            # Save each item separately
            for i, item in enumerate(obj_info["items"]):
                item_key = f"{obj_type}_item_{i}"
                objects_simplified[item_key] = item
        
        # Save grid and metadata
        np.savez(filename, 
                 grid=self.grid, 
                 min_lat=self.min_lat,
                 max_lat=self.max_lat,
                 min_lon=self.min_lon,
                 max_lon=self.max_lon,
                 grid_resolution=self.grid_resolution,
                 corner_coords=np.array(self.corner_coords),
                 objects=objects_simplified)
        
        print(f"Grid data saved to {filename}")
        return filename
    
    @classmethod
    def load_grid_data(cls, filename):
        """Load grid data from a NumPy file"""
        data = np.load(filename, allow_pickle=True)
        
        # Extract corner coordinates
        corner_coords = data['corner_coords'].tolist()
        
        # Create a new PaddockMap instance
        paddock_map = cls(corner_coords, data['grid_resolution'].item())
        
        # Override properties with loaded data
        paddock_map.grid = data['grid']
        paddock_map.min_lat = data['min_lat'].item()
        paddock_map.max_lat = data['max_lat'].item()
        paddock_map.min_lon = data['min_lon'].item()
        paddock_map.max_lon = data['max_lon'].item()
        
        # Load objects dictionary
        objects_dict = data['objects'].item()
        
        # Reconstruct objects
        paddock_map.objects = {}
        
        # First pass: create object types
        for key, value in objects_dict.items():
            if not key.endswith("_item_0") and not key.endswith("_item_1") and not "_item_" in key:
                obj_type = key
                paddock_map.objects[obj_type] = {
                    "color": value["color"],
                    "value": value["value"],
                    "items": []
                }
        
        # Second pass: add items to object types
        for key, value in objects_dict.items():
            if "_item_" in key:
                obj_type = key.split("_item_")[0]
                if obj_type in paddock_map.objects:
                    paddock_map.objects[obj_type]["items"].append(value)
        
        print(f"Grid data loaded from {filename}")
        return paddock_map
    
    def find_path(self, start_lat, start_lon, goal_lat, goal_lon):
        """Find a path from start to goal using A* algorithm"""
        # Convert GPS to grid coordinates
        start = self.gps_to_grid(start_lat, start_lon)
        goal = self.gps_to_grid(goal_lat, goal_lon)
        
        # A* implementation
        open_set = {start}
        closed_set = set()
        
        came_from = {}
        
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        while open_set:
            # Get node with lowest f_score
            current = min(open_set, key=lambda pos: f_score.get(pos, float('inf')))
            
            if current == goal:
                path = self._reconstruct_path(came_from, current)
                return path
            
            open_set.remove(current)
            closed_set.add(current)
            
            # Check neighbors (8-connectivity for smoother paths)
            neighbors = [
                (current[0]+1, current[1]),    # Right
                (current[0]-1, current[1]),    # Left
                (current[0], current[1]+1),    # Up
                (current[0], current[1]-1),    # Down
                (current[0]+1, current[1]+1),  # Diagonal: Up-Right
                (current[0]-1, current[1]+1),  # Diagonal: Up-Left
                (current[0]+1, current[1]-1),  # Diagonal: Down-Right
                (current[0]-1, current[1]-1)   # Diagonal: Down-Left
            ]
            
            for neighbor in neighbors:
                if (neighbor[0] < 0 or neighbor[0] >= self.grid_width or
                    neighbor[1] < 0 or neighbor[1] >= self.grid_height or
                    self.grid[neighbor[1], neighbor[0]] > 0 or  # Any non-zero value is an obstacle
                    neighbor in closed_set):
                    continue
                
                # Calculate movement cost (diagonal moves cost more)
                is_diagonal = abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) > 1
                move_cost = 1.414 if is_diagonal else 1.0  # sqrt(2) for diagonal
                
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor, goal)
        
        # No path found
        return None
    
    def _heuristic(self, a, b):
        """Euclidean distance heuristic for A*"""
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from A* result"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        
        # Convert back to GPS coordinates
        gps_path = [self.grid_to_gps(x, y) for x, y in path]
        return gps_path
    
    def optimize_path(self, path, smoothing_factor=0.5):
        """Smooth the path using path smoothing algorithm"""
        if not path or len(path) <= 2:
            return path
            
        # Convert GPS path to grid path for processing
        grid_path = [self.gps_to_grid(lat, lon) for lat, lon in path]
        smoothed_path = [p for p in grid_path]  # Make a copy
        
        change = tolerance = 0.1
        while change >= tolerance:
            change = 0
            
            # Skip endpoints
            for i in range(1, len(grid_path) - 1): 
                old_x, old_y = smoothed_path[i]
                
                # Pull toward original path
                smoothed_path[i] = (
                    old_x + smoothing_factor * (grid_path[i][0] - old_x),
                    old_y + smoothing_factor * (grid_path[i][1] - old_y)
                )
                
                # Calculate change
                change += abs(old_x - smoothed_path[i][0]) + abs(old_y - smoothed_path[i][1])
        
        # Convert back to GPS coordinates
        return [self.grid_to_gps(int(x), int(y)) for x, y in smoothed_path]




class RobotSimulator:
    def __init__(self, initial_lat, initial_lon, heading_deg=0):
        """
        Initialize robot simulator
        
        Args:
            initial_lat: Initial latitude
            initial_lon: Initial longitude
            heading_deg: Initial heading in degrees (0 = North, 90 = East)
        """
        self.lat = initial_lat
        self.lon = initial_lon
        self.heading = radians(heading_deg)  # Convert to radians for internal use
        
        # Robot physical parameters
        self.wheel_base = 1.0  # meters between front and rear axles
        self.speed = 0.0  # meters per second (positive = forward, negative = backward)
        self.steering_angle = 0.0  # radians (positive = right, negative = left)
        self.max_steering_angle = radians(30)  # maximum steering angle in radians
        self.max_speed = 2.0  # meters per second
        
        # For simulation
        self.last_update_time = time.time()

    def set_speed(self, speed):
        """Set the robot's speed in meters per second"""
        # Limit to max speed
        self.speed = max(min(speed, self.max_speed), -self.max_speed)
    
    def set_steering_angle(self, angle_deg):
        """Set the steering angle in degrees"""
        # Convert to radians and limit to max steering angle
        angle_rad = radians(angle_deg)
        self.steering_angle = max(min(angle_rad, self.max_steering_angle), -self.max_steering_angle)
    
    def update_position(self, elapsed_time):
        """Update position based on current speed, steering angle, and elapsed time"""
        if abs(self.speed) < 0.001:  # Not moving
            return
        
        # Distance traveled
        distance = self.speed * elapsed_time  # meters
        
        # For very small steering angles, assume straight line
        if abs(self.steering_angle) < 0.001:
            # Simple straight-line movement
            dx = distance * sin(self.heading)  # East/West component
            dy = distance * cos(self.heading)  # North/South component
        else:
            # Calculate turning radius
            turning_radius = self.wheel_base / sin(abs(self.steering_angle))
            
            # Calculate change in heading
            dheading = distance / turning_radius
            if self.steering_angle < 0:
                dheading = -dheading
            if self.speed < 0:  # Going backward changes the direction of turn
                dheading = -dheading
            
            # Calculate position change using arc
            dx = turning_radius * (cos(self.heading) - cos(self.heading + dheading))
            dy = turning_radius * (sin(self.heading + dheading) - sin(self.heading))
            
            # Update heading
            self.heading = (self.heading + dheading) % (2 * pi)
        
        # Convert dx, dy to changes in latitude and longitude
        # Approximate conversion from meters to degrees
        earth_radius = 6371000  # meters
        dlat = (dy / earth_radius) * (180 / pi)
        dlon = (dx / earth_radius) * (180 / pi) / cos(radians(self.lat))
        
        # Update position
        self.lat += dlat
        self.lon += dlon
    
    def get_position(self):
        """Get current position (lat, lon) and heading in degrees"""
        return {
            'lat': self.lat,
            'lon': self.lon,
            'heading': degrees(self.heading) % 360
        }
    
    def simulate_step(self):
        """Simulate one time step"""
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update position based on elapsed time
        self.update_position(elapsed_time)
        
        return self.get_position()
    
    def steer_towards_waypoint(self, waypoint_lat, waypoint_lon):
        """Calculate steering and speed to navigate towards waypoint"""
        # Calculate bearing to waypoint
        target_bearing = calculate_bearing(self.lat, self.lon, waypoint_lat, waypoint_lon)
        current_bearing = degrees(self.heading) % 360
        
        # Calculate difference in bearing (shortest path)
        bearing_diff = ((target_bearing - current_bearing + 180) % 360) - 180
        
        # Calculate distance to waypoint
        distance = haversine_distance(self.lat, self.lon, waypoint_lat, waypoint_lon)
        
        # Adjust steering angle based on bearing difference
        # Simple proportional control with maximum angle limit
        steering_angle = max(min(bearing_diff * 0.5, 30), -30)  # Scale factor of 0.5
        
        # Adjust speed based on steering angle and distance to waypoint
        # Slow down for sharp turns and when approaching waypoint
        speed_factor = 1.0 - (abs(steering_angle) / 45.0)  # Reduce speed in turns
        
        # Slow down when approaching waypoint
        approach_factor = min(distance / 5.0, 1.0)  # Start slowing at 5 meters
        
        # Calculate speed (with minimum to prevent stopping)
        speed = max(self.max_speed * speed_factor * approach_factor, 0.5)
        
        return steering_angle, speed

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing from point 1 to point 2 in degrees"""
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    y = sin(lon2 - lon1) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)
    initial_bearing = atan2(y, x)
    
    # Convert to degrees and normalize
    bearing = (degrees(initial_bearing) + 360) % 360
    return bearing

def main():
    # Example paddock corners (latitude, longitude)
    paddock_corners = [
        (37.7749, -122.4194),  # Corner 1
        (37.7749, -122.4154),  # Corner 2
        (37.7739, -122.4154),  # Corner 3
        (37.7739, -122.4194)   # Corner 4
    ]
    
    # Create paddock map with 1-meter grid resolution
    paddock = PaddockMap(paddock_corners, grid_resolution=1.0)
    
    # Add some permanent objects
    # Tree
    paddock.add_circular_object(37.7745, -122.4180, 3.0, name="Tree")
    
    # Water trough
    paddock.add_rectangular_object(37.7743, -122.4170, 37.7741, -122.4165, name="Water Trough")
    
    # Rock pile
    paddock.add_circular_object(37.7740, -122.4185, 2.5, name="Rock Pile")
    
    # Fence post
    paddock.add_point_object(37.7747, -122.4160, name="Gate Post")
    
    # Initial robot position
    initial_position = (37.7747, -122.4184)
    
    # Target position
    target_position = (37.7742, -122.4164)
    
    # Create robot simulator
    robot = RobotSimulator(initial_position[0], initial_position[1], 90)  # Starting heading 90° (East)
    
    # Find path from robot to target
    path = paddock.find_path(*initial_position, *target_position)
    
    if not path:
        print("No path found to target!")
        return
    
    # Optimize the path
    path = paddock.optimize_path(path)
    print(f"Found path with {len(path)} waypoints")
    
    # Main simulation loop
    current_waypoint_index = 0
    waypoint_reached_distance = 3.0  # meters
    
    try:
        print("Starting autonomous navigation...")
        while current_waypoint_index < len(path):
            # Get current waypoint
            waypoint = path[current_waypoint_index]
            
            # Update robot position
            pos = robot.simulate_step()
            current_position = (pos['lat'], pos['lon'])
            
            # Calculate distance to current waypoint
            dist_to_waypoint = haversine_distance(current_position[0], current_position[1],
                                                waypoint[0], waypoint[1])
            
            # Check if waypoint reached
            if dist_to_waypoint < waypoint_reached_distance:
                print(f"Waypoint {current_waypoint_index+1}/{len(path)} reached!")
                current_waypoint_index += 1
                
                # If all waypoints reached
                if current_waypoint_index >= len(path):
                    print("Target destination reached!")
                    break
            
            # Update steering and speed to navigate towards current waypoint
            steering_angle, speed = robot.steer_towards_waypoint(waypoint[0], waypoint[1])
            robot.set_steering_angle(steering_angle)
            robot.set_speed(speed)
            
            # Print status every second
            if int(time.time()) % 1 == 0:
                print(f"Position: ({pos['lat']:.6f}, {pos['lon']:.6f}), " +
                      f"Heading: {pos['heading']:.1f}°, " +
                      f"Steering: {steering_angle:.1f}°, " +
                      f"Speed: {speed:.1f} m/s, " +
                      f"Distance to waypoint: {dist_to_waypoint:.1f}m")
                
                # Check if inside boundary
                if not paddock.is_inside_boundary(*current_position):
                    print("WARNING: Robot outside paddock boundary!")
                
                # Visualize the map
                remaining_path = path[current_waypoint_index:]
                paddock.plot_map(current_position, target_position, remaining_path, 
                               save=True, show=False)
            
            # Delay for simulation
            time.sleep(0.1)
        
        # Final map visualization
        paddock.plot_map(current_position, target_position, None, 
                       save=True, show=True)
        print("Simulation complete.")
            
    except KeyboardInterrupt:
        print("Simulation stopped.")

if __name__ == "__main__":
    main()