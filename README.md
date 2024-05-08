# Mobile Robot Path Planning with Nav2 and Dijkstra’s Algorithm

This project demonstrates the implementation of Dijkstra’s Algorithm for mobile robot path planning using the Nav2 framework in ROS2. By swapping Nav2's default planner for a custom Dijkstra implementation, this exercise highlights the practical application of a well-known pathfinding algorithm to facilitate autonomous navigation.

## Project Structure

- **src/nav2_dijkstra_planner/**: Contains the C++ implementation of Dijkstra's Algorithm for pathfinding.
- **config/**: Holds the Nav2 configuration files required to replace the default planner with the custom Dijkstra planner.
- **launch/**: Provides the launch files to initialize the Gazebo simulation and the Nav2 stack.
- **neo_simulation2/**: Simulation models and world files for Gazebo.

## Getting Started

To get started with this project, ensure you have ROS2 and Nav2 properly installed in your workspace. Follow the steps below to build and run the project.

### Prerequisites

- ROS2 Foxy or later
- Nav2 Navigation Stack
- Gazebo Simulator

### Installation

1. **Clone the Repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/MiguelSolisSegura/nav2_dijkstra_planner.git
   ```

2. **Build the Workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### Running the Simulation

1. **Launch Gazebo:**
   ```bash
   export GAZEBO_MODEL_PATH=/home/user/ros2_ws/src/nav2_dijkstra_planner/neo_simulation2/models:/home/user/ros2_ws/src
   ros2 launch neo_simulation2 simulation.launch.py
   ```

2. **Launch Nav2:**
   ```bash
   ros2 launch neo_nav2 neo_nav2_full.launch.xml
   ```

3. **Set a Goal in Rviz2:**
   - Open Rviz2 and use the "2D Nav Goal" tool to set a navigation target for the robot.
