"""
Robot Testing Script with Paddock Maps
This script demonstrates how to load saved paddock maps and use them
for robot testing with simulated navigation.
"""
from RobotSimulator import RobotSimulator, load_paddock_map, test_robot_navigation

def main():
    """Main function to test robot navigation on saved maps"""
    # Map options
    map_options = [
        {
            "name": "Simple Rectangular Paddock",
            "filename": "saved_maps/simple_rectangular_paddock.npz",
            "start_pos": (-33.89004, 151.18905)  # Example starting position
        },
        {
            "name": "Complex Paddock",
            "filename": "saved_maps/complex_paddock.npz",
            "start_pos": (-33.885, 151.1846)  # Example starting position
        },
        {
            "name": "Obstacle Course",
            "filename": "saved_maps/obstacle_course.npz",
            "start_pos": (-33.89548, 151.19408)  # Example starting position
        }
    ]
   
    # Print menu
    print("=== ROBOT TESTING SCRIPT ===")
    print("Choose a map to test robot navigation:")
   
    for i, option in enumerate(map_options):
        print(f"{i+1}. {option['name']}")
   
    try:
        choice = int(input("\nEnter your choice (1-3): "))
        if 1 <= choice <= len(map_options):
            selected = map_options[choice-1]
            print(f"\nSelected: {selected['name']}")
           
            # Test navigation on the selected map and save results to robot_maps directory
            test_robot_navigation(selected['filename'], selected['start_pos'])
        else:
            print("Invalid choice!")
    except ValueError:
        print("Please enter a number!")

if __name__ == "__main__":
    main()