# Autonomous Waypoint Navigation System Documentation

## Overview
This documentation covers the implementation of an autonomous waypoint navigation system for a robot operating in a paddock environment. The system simulates GPS updates and uses these updates to guide a robot through a series of waypoints while maintaining appropriate steering angles and forward/backward movement.

## System Components

### 1. RobotSimulator Class
The primary simulation engine that models robot movement, position updates, and navigation control.

#### Key Properties
- `lat`, `lon`: Current position in latitude/longitude coordinates
- `heading`: Current heading in radians (0 = North, π/2 = East)
- `wheel_base`: Distance between front and rear axles in meters
- `speed`: Current speed in meters per second (positive = forward, negative = backward)
- `steering_angle`: Current steering angle in radians (positive = right, negative = left)
- `max_steering_angle`: Maximum allowed steering angle in radians
- `max_speed`: Maximum allowed speed in meters per second

#### Core Methods

##### `set_speed(speed)`
Sets the robot's linear speed, constrained by `max_speed`.

- **Parameters**:
  - `speed`: Target speed in meters per second
- **Returns**: None
- **Constraints**: Value is clamped between `-max_speed` and `max_speed`

##### `set_steering_angle(angle_deg)`
Sets the robot's steering angle in degrees, converted to radians internally.

- **Parameters**:
  - `angle_deg`: Target steering angle in degrees
- **Returns**: None
- **Constraints**: Value is clamped between `-max_steering_angle` and `max_steering_angle`

##### `update_position(elapsed_time)`
Updates the robot's position based on current steering angle, speed, and elapsed time.

- **Parameters**:
  - `elapsed_time`: Time elapsed since the last update in seconds
- **Returns**: None
- **Implementation Details**:
  - For straight-line movement (very small steering angle):
    - Calculates position changes along heading vector
  - For curved movement:
    - Calculates turning radius based on wheel base and steering angle
    - Computes change in heading based on distance traveled along arc
    - Updates position using arc geometry
  - Converts linear distance to lat/lon changes using Earth's radius
  - Updates the robot's current lat/lon and heading

##### `get_position()`
Returns the current position and heading information.

- **Parameters**: None
- **Returns**: Dictionary with keys:
  - `lat`: Current latitude
  - `lon`: Current longitude
  - `heading`: Current heading in degrees (0-360)

##### `simulate_step()`
Performs a single simulation step by calculating time elapsed since last update and updating position.

- **Parameters**: None
- **Returns**: Current position dictionary (same as `get_position()`)
- **Implementation Details**:
  - Calculates elapsed time since last simulation step
  - Updates position using `update_position()`
  - Returns current position data

##### `steer_towards_waypoint(waypoint_lat, waypoint_lon)`
Calculates the appropriate steering angle and speed to navigate toward a waypoint.

- **Parameters**:
  - `waypoint_lat`: Target waypoint latitude
  - `waypoint_lon`: Target waypoint longitude
- **Returns**: Tuple of `(steering_angle, speed)`
- **Implementation Details**:
  - Calculates bearing to waypoint
  - Computes bearing difference with current heading
  - Uses proportional control to set steering angle
  - Adjusts speed based on:
    - Turning severity (slower in sharp turns)
    - Distance to waypoint (slower when approaching)
  - Maintains minimum speed for continuous movement

### 2. Navigation Helper Functions

#### `calculate_bearing(lat1, lon1, lat2, lon2)`
Calculates the bearing from one point to another in degrees.

- **Parameters**:
  - `lat1`, `lon1`: Start point coordinates
  - `lat2`, `lon2`: End point coordinates
- **Returns**: Bearing in degrees (0-360, where 0 = North, 90 = East)
- **Implementation Details**:
  - Converts input coordinates to radians
  - Uses the haversine formula to calculate the bearing
  - Normalizes result to 0-360 degree range

#### `haversine_distance(lat1, lon1, lat2, lon2)`
Calculates the great-circle distance between two points on Earth's surface.

- **Parameters**:
  - `lat1`, `lon1`: Start point coordinates
  - `lat2`, `lon2`: End point coordinates
- **Returns**: Distance in meters
- **Implementation Details**:
  - Uses the haversine formula for spherical distance calculation
  - Accounts for Earth's radius (6371 km)

### 3. Main Program Flow

The main program orchestrates the simulation with the following steps:

1. **Initialization**:
   - Create paddock environment with boundaries and obstacles
   - Set initial robot position and target destination
   - Generate optimized path using `PaddockMap.find_path()`

2. **Navigation Loop**:
   - For each waypoint in the path:
     - Calculate steering and speed using `steer_towards_waypoint()`
     - Update robot position using `simulate_step()`
     - Check if waypoint is reached (within threshold distance)
     - Move to next waypoint when current one is reached
     - Periodically visualize position and path

3. **Termination**:
   - End simulation when all waypoints are reached
   - Display final map visualization

## Physics Model

### Position Updates
The robot's position is updated based on a bicyle model with the following characteristics:

1. **Straight Movement**:
   - When steering angle is very small, position updates along a straight line
   - Change in position is calculated as: `distance * [sin(heading), cos(heading)]`

2. **Curved Movement**:
   - For non-zero steering angles, position follows an arc
   - Turning radius (R) = `wheel_base / sin(|steering_angle|)`
   - Change in heading = `distance / R`
   - Position changes calculated using circular arc geometry

3. **Coordinate Conversion**:
   - Linear displacements are converted to latitude/longitude using:
     - `dlat = (dy / earth_radius) * (180 / π)`
     - `dlon = (dx / earth_radius) * (180 / π) / cos(lat)`

### Control System

1. **Steering Control**:
   - Uses proportional control based on bearing difference
   - `steering_angle = bearing_difference * gain_factor`
   - Limited to maximum steering angle (±30 degrees)

2. **Speed Control**:
   - Base speed determined by maximum speed setting
   - Reduced in proportion to steering angle magnitude (slower in turns)
   - Reduced when approaching waypoints (gradual deceleration)
   - Maintains minimum speed to prevent stopping

## Usage Instructions

1. **Setup**:
   - Define paddock boundaries and obstacles using `PaddockMap` class
   - Set initial robot position and heading
   - Define target destination position

2. **Path Planning**:
   - Use `paddock.find_path()` to generate a path from start to destination
   - Optimize path using `paddock.optimize_path()` to remove redundant waypoints

3. **Simulation**:
   - Run the main loop to start autonomous navigation
   - System will automatically navigate through waypoints
   - Monitor progress through console output and visualizations

4. **Monitoring**:
   - Position, heading, steering angle, and speed information displayed periodically
   - Map visualizations show current position and remaining path
   - Warnings displayed if robot leaves paddock boundary

## Customization

### Robot Parameters
The following parameters can be adjusted to modify robot behavior:

- `wheel_base`: Affects turning radius and maneuverability
- `max_steering_angle`: Maximum allowed steering angle in radians
- `max_speed`: Maximum allowed speed in meters per second
- `waypoint_reached_distance`: Distance threshold to consider a waypoint reached

### Control Parameters
The steering controller can be tuned by modifying:

- Steering gain factor (currently 0.5) to adjust steering sensitivity
- Speed reduction factors for turns and waypoint approach
- Minimum speed threshold to prevent stopping

## Limitations and Future Improvements

1. **Simulation Accuracy**:
   - Current model simplifies real-world physics
   - Does not account for terrain, friction, or motor dynamics
   - GPS updates are assumed to be perfect without error

2. **Path Optimization**:
   - Current implementation uses basic path optimization
   - Could be improved with spline-based smoothing
   - Dynamic replanning not implemented if obstacles move

3. **Control Enhancements**:
   - Current control uses simple proportional steering
   - Could be enhanced with PID control for better stability
   - Acceleration/deceleration modeling could be improved

4. **Real-world Integration**:
   - System designed for simulation only
   - Would need interfaces to actual GPS and motor controllers
   - Error handling for sensor noise and control delays