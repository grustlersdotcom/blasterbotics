"""
PaddockMap class for representing and managing the paddock environment.
"""
import numpy as np
import os
from datetime import datetime
from geo_utils import haversine_distance
from math import cos,radians

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