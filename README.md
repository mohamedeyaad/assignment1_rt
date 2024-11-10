# Assignment 1 - Research Track I  
## Project Overview
This repository contains `assignment1_rt`, a ROS package developed as part of Assignment 1 for the *Research Track I* course. The package consists of two nodes (`ui` and `distance`) implemented in both Python and C++. 

- **UI Node (`ui`)**: Enables user interaction for turtle control in the simulation environment.
- **Distance Node (`distance`)**: Monitors the distance between turtles and ensures safety boundaries.

## Nodes Description

### UI Node (`ui`)
- **Purpose**: Provides a command-line interface to control two turtles (`turtle1` and `turtle2`).
- **Features**:
  - Spawns a new turtle (`turtle2`).
  - Allows users to select the turtle to control and set the velocity.
  - Commands run for 1 second before the turtle stops, allowing repeated user inputs.
- **Execution Options**: Implemented in both Python (`ui.py`) and C++ (`ui.cpp`).

### Distance Node (`distance`)
- **Purpose**: Continuously calculates and publishes the distance between `turtle1` and `turtle2` on a ROS topic.
- **Features**:
  - Publishes distance between the 2 turtles on `/turtle_distance` topic using `std_msgs/Float32` message type.
  - Stops the moving turtle upon reaching a proximity threshold of 1 unit from the other turtle.
  - Stops the turtle if it nears simulation boundaries (x or y > 10.0 or x or y < 1.0).
- **Implementation**: Implemented  in both Python (`distance.py`) and C++ (`distance.cpp`).

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

## Running the Nodes
1. **Start the turtlesim environment**:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

2. **Run the UI Node**:
   - **Python**: `rosrun assignment1_rt ui.py`
   - **C++**: `rosrun assignment1_rt ui`

3. **Run the Distance Node**:
   - **Python**: `rosrun assignment1_rt distance.py`
   - **C++**: `rosrun assignment1_rt distance`

## Repository Structure
```
assignment1_rt/
├── src/
│   ├── ui.cpp             # UI node (C++)
│   ├── distance.cpp       # Distance node (C++)
├── scripts/
│   ├── ui.py              # UI node (Python)
│   ├── distance.py        # Distance node (Python)
├── CMakeLists.txt
├── package.xml
└── README.md
```
