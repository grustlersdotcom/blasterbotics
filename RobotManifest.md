# Robot Prototype Design and Software

This robot was developed on a pretty tight budget, so naturally, a few corners had to be cut along the way. Over the course of development, a lot of mechanical and power system changes happened, which ended up breaking or invalidating a lot of the code I originally wrote. As a result, a good chunk of the original software had to be scrapped or reworked, and unfortunately, I didn’t get to move past basic hardware testing.

The Python code included here simulates what the robot is supposed to do. Right now, the physical robot isn’t fully operational — it doesn’t even turn — and several control systems are incomplete. That said, the Jetson board has working code for most of the hardware components like GPS, LiDAR, and the rest. While it’s not all wired together, the groundwork is there for someone to pick up and move forward.

## Written Code
### geo_utils

Contains functions to convert GPS data (from the u-blox module) into grid coordinates. This is how the robot understands where it is and navigates around. Also useful for calculating distances.

### PaddockMap
A class that handles loading, saving, and representing paddock maps. Saved maps go into the saved_maps folder. There's room to add logic for selecting a map based on current GPS position — shouldn’t be too hard, maybe just a JSON metadata file to pair maps with locations.

### RobotSimulator
This is where most of the testing/visualization tools live. It stores simulated robot maps in robot_maps and has functions to help visualize movement and state. Very helpful when testing without hardware.

### StallSweep
Runs a Roomba-style sweeping pattern across a paddock, carving out lanes and updating waypoints as it goes.

### TalkToArduino
Small helper library to communicate with the Arduino controller over I2C.

### Pathfinding_demo_main
The main demo script we showed at the showcase — pulls together a bunch of the above classes and shows how they interact. Good starting point to get a feel for how it’s meant to work.

## Future Work
Originally, the plan was to control everything using Python on the Jetson, but that fell through near the end due to the hardware changes. So here’s what still needs doing:

    Individual testing: All the existing scripts/modules should be tested independently to make sure they still work on their own.

    Dumping system: After every three scoops, the robot is supposed to head to a dump location, drop the payload, and then resume where it left off. That logic was never implemented. Dump location context can be stored in the paddock maps or in an external metadata file — up to whoever picks this up next.

    Resuming path after dump: This was where I got stuck. The robot needs to remember its last location before dumping and return to that same point to keep going. Shouldn’t be too hard logically, but I ran out of time before figuring it out.

    Arduino + control tuning: The hardware got swapped out after I graduated (wheels, power system, etc.), so the control and Arduino code will need to be reworked/tuned to match the new setup.

    Control structure: I set this up using global heading and forward/backward state variables. The idea was that subsystems like LiDAR or camera vision could change those values when they needed control. That system is in place but wasn’t fully tested due to other issues.

    Vision code: Lives elsewhere (not in this repo). The objects identified by the camera just need some basic linear algebra to calculate the angle from the robot’s forward vector, then pass that info to the control system.

    LiDAR: Right now, it's mostly configured as a basic stop interrupt. It needs logic to help the robot re-route around obstacles. No idea how to do that part — someone smarter can figure it out.

    PID: Could add one for smoother control, but you might be fine without it depending on how janky the final system is.

    ROS compatibility: Everything written here can be translated into ROS without much trouble. I didn’t get to it, but the structure and logic should port over cleanly.