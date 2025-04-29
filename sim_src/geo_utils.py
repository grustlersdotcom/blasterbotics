"""
Geographical utility functions for the paddock robot project.
"""
import numpy as np
from math import radians, sin, cos, sqrt, atan2, degrees, pi

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate distance between two GPS points in meters using the Haversine formula
    
    Args:
        lat1, lon1: GPS coordinates of first point
        lat2, lon2: GPS coordinates of second point
        
    Returns:
        Distance in meters
    """
    R = 6371000  # Earth radius in meters
    
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2 - lat1)
    delta_lambda = radians(lon2 - lon1)
    
    a = sin(delta_phi/2)**2 + cos(phi1) * cos(phi2) * sin(delta_lambda/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate bearing from point 1 to point 2 in degrees
    
    Args:
        lat1, lon1: GPS coordinates of start point
        lat2, lon2: GPS coordinates of target point
        
    Returns:
        Bearing in degrees (0 = North, 90 = East, etc.)
    """
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    y = sin(lon2 - lon1) * cos(lat2)
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1)
    initial_bearing = atan2(y, x)
    
    # Convert to degrees and normalize
    bearing = (degrees(initial_bearing) + 360) % 360
    return bearing

def gps_to_xy(lat, lon, origin_lat, origin_lon):
    """
    Convert GPS coordinates to meters from origin point (approximate)
    
    Args:
        lat, lon: GPS coordinates
        origin_lat, origin_lon: Origin point GPS coordinates
        
    Returns:
        Tuple (x, y) representing meters east and north of origin
    """
    x_meters = haversine_distance(origin_lat, origin_lon, origin_lat, lon)
    y_meters = haversine_distance(origin_lat, origin_lon, lat, origin_lon)
    
    # Adjust for direction (east/west and north/south)
    if lon < origin_lon:
        x_meters = -x_meters
    if lat < origin_lat:
        y_meters = -y_meters
    
    return (x_meters, y_meters)

def xy_to_gps(x_meters, y_meters, origin_lat, origin_lon):
    """
    Convert meters from origin to GPS coordinates (approximate)
    
    Args:
        x_meters: Meters east of origin (negative for west)
        y_meters: Meters north of origin (negative for south)
        origin_lat, origin_lon: Origin point GPS coordinates
        
    Returns:
        Tuple (lat, lon) representing GPS coordinates
    """
    earth_radius = 6371000  # meters
    
    # Calculate new position
    lon = origin_lon + (x_meters / (earth_radius * cos(radians(origin_lat)))) * (180 / pi)
    lat = origin_lat + (y_meters / earth_radius) * (180 / pi)
    
    return (lat, lon)