# ROS2 Foxy Maze Solver

This package is a ROS2 implementation of the A* algorithm for solving a maze in simulation using Gazebo. It is designed to work with the ROS2 Foxy distribution and can be used with any simulated overhead camera that is compatible with ROS2.

## Requirements
- ROS2 Foxy
- Gazebo
- A simulated overhead camera compatible with ROS2

## Installation

1. Clone this repository into your ROS2 workspace
  ```bash
      cd <your_ros2_workspace>/src
      git clone https://github.com/<your_username>/ros2_foxy_maze_solver.git
  ```

2. Build the package
  ```bash
      cd <your_ros2_workspace>
      colcon build
  ```

3. Configure the package to use your simulated overhead camera.

## Usage

1. Launch the maze_solver node and the gazebo simulation
  ```bash
      ros2 launch ros2_foxy_maze_solver maze_solver_simulation.launch.py
  ```

2. The node will start processing the images from the simulated camera and will publish the solution path to the /maze_solver/path topic.

## Known issues
Please report any issue or bug you encounter in the issue section.

## Contribution
Any contribution is welcome, feel free to fork the repo and open a pull request.

## License
This package is licensed under the MIT License.
