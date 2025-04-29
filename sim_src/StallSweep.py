def generate_coverage_path(self, start_lat, start_lon, lane_width=2.0):
    """
    Generate a coverage path that covers the entire paddock in a lawnmower pattern
    
    Args:
        start_lat: Starting latitude
        start_lon: Starting longitude
        lane_width: Width between parallel paths in meters
        
    Returns:
        List of (lat, lon) coordinates forming the coverage path
    """
    # Convert lane width from meters to grid cells
    lane_width_cells = max(1, int(lane_width / self.grid_resolution))
    
    # Create a copy of the grid to mark visited areas
    visited = np.zeros_like(self.grid)
    
    # Mark obstacles and boundaries as visited
    visited[self.grid > 0] = 1
    
    # Get starting position in grid coordinates
    start_grid = self.gps_to_grid(start_lat, start_lon)
    current_pos = start_grid
    
    # Direction vectors (right, down, left, up)
    directions = [(1, 0), (0, -1), (-1, 0), (0, 1)]
    current_dir = 0  # Start moving right
    
    # Path as grid coordinates
    grid_path = [current_pos]
    
    # Mark starting position as visited
    if 0 <= current_pos[1] < visited.shape[0] and 0 <= current_pos[0] < visited.shape[1]:
        visited[current_pos[1], current_pos[0]] = 1
    
    # Continue until we've covered the entire accessible area
    stuck_count = 0
    max_stuck = 4  # If we can't move in any direction after trying all, we're done
    
    while stuck_count < max_stuck:
        # Try to move in current direction
        next_pos = (current_pos[0] + directions[current_dir][0] * lane_width_cells,
                   current_pos[1] + directions[current_dir][1] * lane_width_cells)
        
        # Check if next position is valid (within bounds and not an obstacle)
        if (0 <= next_pos[1] < visited.shape[0] and 
            0 <= next_pos[0] < visited.shape[1] and 
            visited[next_pos[1], next_pos[0]] == 0):
            
            # Draw a line between current_pos and next_pos
            x0, y0 = current_pos
            x1, y1 = next_pos
            
            # Simple Bresenham's line algorithm
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
            
            # Add points along the line to path
            line_points = []
            for x in range(x0, x1 + 1):
                if is_steep:
                    if 0 <= y < visited.shape[1] and 0 <= x < visited.shape[0]:
                        line_points.append((y, x))
                        visited[x, y] = 1
                else:
                    if 0 <= y < visited.shape[0] and 0 <= x < visited.shape[1]:
                        line_points.append((x, y))
                        visited[y, x] = 1
                
                error -= dy
                if error < 0:
                    y += y_step
                    error += dx
            
            # Add line points to path (excluding start which is already there)
            grid_path.extend(line_points[1:] if line_points else [])
            current_pos = next_pos
            
            # Reset stuck counter
            stuck_count = 0
        else:
            # Try to turn
            current_dir = (current_dir + 1) % 4
            stuck_count += 1
    
    # Convert grid path to GPS coordinates
    gps_path = [self.grid_to_gps(x, y) for x, y in grid_path]
    
    # Optimize path (remove redundant points)
    optimized_path = self._optimize_coverage_path(gps_path)
    
    return optimized_path

def _optimize_coverage_path(self, path, tolerance=1.0):
    """
    Optimize the coverage path by removing redundant points
    
    Args:
        path: List of (lat, lon) coordinates
        tolerance: Distance tolerance in meters
        
    Returns:
        Optimized path
    """
    if not path or len(path) <= 2:
        return path
    
    # Keep first point
    optimized = [path[0]]
    
    for i in range(1, len(path) - 1):
        # Calculate distance between point i-1 and i+1
        dist = haversine_distance(path[i-1][0], path[i-1][1], 
                                path[i+1][0], path[i+1][1])
        
        # Calculate distance from point i to line formed by i-1 and i+1
        # Using cross product approximation for perpendicular distance
        a = haversine_distance(path[i-1][0], path[i-1][1], 
                             path[i][0], path[i][1])
        b = haversine_distance(path[i][0], path[i][1], 
                             path[i+1][0], path[i+1][1])
        c = haversine_distance(path[i-1][0], path[i-1][1], 
                             path[i+1][0], path[i+1][1])
        
        # Semi-perimeter
        s = (a + b + c) / 2
        
        # Area of triangle using Heron's formula
        try:
            area = sqrt(s * (s-a) * (s-b) * (s-c))
        except ValueError:
            # Handle floating point errors
            area = 0
            
        # Height of triangle (perpendicular distance)
        if c > 0:
            height = 2 * area / c
        else:
            height = 0
        
        # If not collinear and perpendicular distance is greater than tolerance, keep it
        if height > tolerance:
            optimized.append(path[i])
    
    # Always keep the last point
    optimized.append(path[-1])
    return optimized

def clean_paddock(self, start_lat, start_lon, lane_width=2.0, animate=False):
    """
    Generate and simulate a complete paddock cleaning route
    
    Args:
        start_lat: Starting latitude
        start_lon: Starting longitude
        lane_width: Width between parallel paths in meters
        animate: Whether to animate the cleaning process
        
    Returns:
        Coverage path as list of GPS coordinates
    """
    # Generate coverage path
    print("Generating coverage path...")
    coverage_path = self.generate_coverage_path(start_lat, start_lon, lane_width)
    
    if not coverage_path:
        print("Failed to generate coverage path!")
        return None
    
    print(f"Coverage path generated with {len(coverage_path)} waypoints")
    
    # Display the full path
    self.plot_map(
        robot_position=(start_lat, start_lon),
        path=coverage_path,
        save=True, 
        show=not animate  # Only show final map if not animating
    )
    
    if animate:
        # Create robot simulator
        robot = RobotSimulator(start_lat, start_lon)
        
        # Main simulation loop
        current_waypoint_index = 0
        waypoint_reached_distance = lane_width / 2  # meters
        
        try:
            print("Starting automated paddock cleaning...")
            while current_waypoint_index < len(coverage_path):
                # Get current waypoint
                waypoint = coverage_path[current_waypoint_index]
                
                # Update robot position
                pos = robot.simulate_step()
                current_position = (pos['lat'], pos['lon'])
                
                # Calculate distance to current waypoint
                dist_to_waypoint = haversine_distance(
                    current_position[0], current_position[1],
                    waypoint[0], waypoint[1]
                )
                
                # Check if waypoint reached
                if dist_to_waypoint < waypoint_reached_distance:
                    print(f"Waypoint {current_waypoint_index+1}/{len(coverage_path)} reached!")
                    current_waypoint_index += 1
                    
                    # If all waypoints reached
                    if current_waypoint_index >= len(coverage_path):
                        print("Paddock cleaning complete!")
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
                    
                    # Visualize the map
                    remaining_path = coverage_path[current_waypoint_index:]
                    self.plot_map(
                        current_position, 
                        None,  # No target position needed
                        remaining_path, 
                        save=True, 
                        show=False
                    )
                
                # Delay for simulation
                time.sleep(0.1)
            
            # Final map visualization
            self.plot_map(
                current_position, 
                None,
                None, 
                save=True, 
                show=True
            )
            print("Simulation complete.")
                
        except KeyboardInterrupt:
            print("Simulation stopped.")
    
    return coverage_path