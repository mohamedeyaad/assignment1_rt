# Assignment 1 - Research Track I  
## Project Overview
This repository contains `assignment1_rt`, a ROS package developed as part of Assignment 1 for the *Research Track I* course. The package consists of two nodes (`ui` and `distance`) implemented in both Python and C++. 

- **UI Node (`ui`)**: Enables user interaction for turtle control in the simulation environment.
- **Distance Node (`distance`)**: Monitors the distance between turtles and ensures safety boundaries.

## Nodes Description

### UI Node (`ui`)
- **Purpose**: Provides a command-line interface to control two turtles (`turtle1` and `turtle2`).
- **Features**:
  - Spawns a new turtle (`turtle2`) with configurable initial parameters.
  - Allows users to select the turtle to control and set the velocity.
  - Commands run for 1 second before the turtle stops, allowing repeated user inputs.
- **Execution Options**: Implemented in both Python (`ui.py`) and C++ (`ui.cpp`).

### Distance Node (`distance`)
- **Purpose**: Continuously calculates and publishes the distance between `turtle1` and `turtle2` on a ROS topic.
- **Features**:
  - Continuously publishes the distance between `turtle1` and `turtle2` on the `/turtle_distance` topic using `std_msgs/Float32` message type.
  - Halts the moving turtle upon reaching a proximity threshold (configurable via the YAML file).
  - Enforces boundary constraints, stopping the turtle if its position exceeds simulation limits.
- **Implementation**: Available in both Python (`distance.py`) and C++ (`distance.cpp`).

## Configuration
A YAML configuration file is included in the `config/` folder to define parameters for the turtles and safety boundaries:
- **File**: `config/params.yaml`
- **Parameters**:
  - `turtle2_pose`: Sets the initial position (`x`, `y`) and orientation (`theta`) of `turtle2`.
  - `distance_threshold`: Distance threshold to stop the moving turtle.
  - `upper_boundary_limit` and `lower_boundary_limit`: Boundary limits to prevent turtles from moving out of bounds.

> **Note**: The YAML configuration file is loaded within the launch file. To run the Python nodes with parameters, use the provided launch file. Running the Python nodes without this file may result in missing parameters. 

## Launching the Project
A launch file is provided to streamline project execution with parameters pre-loaded.

1. **Launch Command** (for Python nodes):
   ```bash
   roslaunch assignment1_rt demo.launch
   ```
   This command launches both the `ui` and `distance` nodes using the Python implementations, while loading parameters from the configuration file.

2. **Running C++ Nodes**: If you prefer to run the C++ versions of the nodes (which do not require external parameters), you may execute them directly without the launch file:
   ```bash
   rosrun assignment1_rt ui
   rosrun assignment1_rt distance
   ```

## Installation and Setup
1. **Clone the Repository**:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-link>
   cd ~/catkin_ws
   catkin_make
   ```
2. **Source the Workspace**:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

## Repository Structure
```
assignment1_rt/
├── config/
│   └── params.yaml               # YAML configuration file for turtles
├── launch/
│   └── demo.launch               # Launch file to run nodes
├── src/
│   ├── ui.cpp                    # UI node (C++)
│   ├── distance.cpp              # Distance node (C++)
├── scripts/
│   ├── ui.py                     # UI node (Python)
│   ├── distance.py               # Distance node (Python)
├── CMakeLists.txt
├── package.xml
└── README.md
```