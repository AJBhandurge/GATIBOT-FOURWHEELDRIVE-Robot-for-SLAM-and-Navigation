# Autonomous Explorer Node for Frontier Exploration

This repository contains a ROS 2 package implementing an autonomous frontier exploration algorithm using Nav2. The Explorer Node subscribes to a map, detects frontiers, and sends navigation goals to explore the environment.

---

https://github.com/user-attachments/assets/08ac6c13-06b5-48d4-823c-2e0e4aa61169

## Features

- **Frontier Detection**: Automatically detects frontiers (unknown areas) in the map.
- **Autonomous Navigation**: Uses Nav2's `NavigateToPose` action to navigate to frontiers.
- **Dynamic Goal Selection**: Chooses the closest unexplored frontier for efficient exploration.
- **ROS 2-Based**: Compatible with ROS 2 (tested on Humble or Foxy distribution).
- **Customizable Timer**: Adjust the exploration frequency as needed.

---

## Requirements

- ROS 2 (Humble/Foxy)
- Python 3
- TurtleBot3 packages installed
- Nav2 installed and configured for your robot
- SLAM Toolbox installed
- `numpy` Python library

---

## Setup

1. Clone the repository into your ROS 2 workspace:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2.git
    cd ~/ros2_ws
    colcon build
    ```

2. Install dependencies:

    ```bash
    pip install numpy
    ```

3. Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

---

## Testing with TurtleBot3

Follow these steps to test the Explorer Node with TurtleBot3 in a Gazebo simulation:

1. Launch the TurtleBot3 world in Gazebo:

    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2. Start the Nav2 stack:

    ```bash
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
    ```

3. Launch SLAM Toolbox for mapping:

    ```bash
    ros2 launch slam_toolbox online_async_launch.py
    ```

4. Launch RViz for visualization:

    ```bash
    ros2 launch nav2_bringup rviz_launch.py
    ```

5. Run the Explorer Node:

    ```bash
    ros2 run custom_explorer explorer
    ```

---

## How It Works

1. **Map Subscription**: Subscribes to the `/map` topic to receive occupancy grid maps.
2. **Frontier Detection**: Identifies free cells adjacent to unknown areas as frontiers.
3. **Navigation**: Sends goals to Nav2's `NavigateToPose` action server for autonomous navigation to the closest frontier.
4. **Dynamic Goal Selection**: Continuously updates and selects frontiers during exploration.

---

## Code Structure

- `explorer.py`: Main node for detecting frontiers and sending navigation goals.
- `requirements.txt`: Python dependencies.
- `README.md`: Project documentation.

---

## Example Workflow

1. **Start the TurtleBot3 simulation environment**.
2. **Run the Explorer Node** as shown in the "Testing with TurtleBot3" section.
3. **Visualize progress in RViz** as the robot autonomously explores the environment.

---

## Future Improvements

- Add support for multi-robot exploration.
- Implement a more advanced frontier selection algorithm.
- Optimize for larger environments.

---

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

---


## Acknowledgments

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [Nav2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://www.turtlebot.com/)
