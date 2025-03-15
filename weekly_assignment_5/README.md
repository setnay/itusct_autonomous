# Turtlesim Path Planner

This project demonstrates a simple path planning and control system using ROS2 and the turtlesim simulator. The system consists of two main nodes:

- **Turtle Controller** (`simple_path_planner.cpp`): This node listens for the turtle’s pose and the current goal, computes the required linear and angular velocities using a proportional controller, and publishes velocity commands to control the turtle.
- **Goal Publisher** (`goal_generator.cpp`): This node generates random goal points within a safe environment (e.g., inside the turtlesim window) ensuring that the generated goal is at least half the window size away from the turtle's initial pose. When the turtle reaches a goal, a new goal is automatically published.

A Python launch file (`planner_launch.py`) is provided to start the turtlesim node, the Turtle Controller node, and the Goal Publisher node together.

## Features

- **Goal-Driven Motion:** The controller uses a simple proportional control algorithm. It calculates the difference between the current turtle pose and the goal to determine the appropriate linear and angular velocities.
- **Shortest-Path Turning:** The angular error is normalized between -π and π, ensuring that the turtle always takes the shortest rotational path toward the goal.
- **Boundary Safety:** The Turtle Controller includes logic to reduce or stop forward motion if the turtle is too close to the window boundaries (e.g., [1.0, 10.0] for turtlesim) to prevent collisions.
- **Dynamic Goal Generation:** When the turtle reaches its current goal (within a threshold distance), the Goal Publisher generates and publishes a new goal that is at least a minimum distance away from its initial pose.
- **ROS2 Launch File:** The `planner_launch.py` file simplifies starting all the nodes needed to run the system.

## Project Structure
```
turtlesim_path_planner/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── planner_launch.py
└── src/
    ├── simple_path_planner.cpp  # Turtle Controller node
    └── goal_generator.cpp       # Goal Publisher node
 ```

### File Details

- **simple_path_planner.cpp**  
  Implements the `TurtleController` node. It:
  - Subscribes to `/turtle1/pose` for the current pose.
  - Subscribes to `/goal` for the target point.
  - Uses a proportional control scheme:
    - Computes the linear velocity as the minimum between a set maximum value and the distance to the goal.
    - Computes the angular velocity as the product of an angular gain and the normalized angular error.
  - Checks if the turtle is near the boundaries (using a defined safe margin) to avoid forward motion that might cause collisions.
  - Stops the turtle when the goal is reached.

- **goal_generator.cpp**  
  Implements the `GoalPublisher` node. It:
  - Subscribes to `/turtle1/pose` to record the turtle’s initial pose.
  - Generates random goal points within a safe area ([1.0, 10.0] by default) ensuring each new goal is at least a minimum distance (approximately 5.5 units) away from the initial pose.
  - Publishes new goals on the `/goal` topic when the turtle has reached the current goal.

- **planner_launch.py**  
  A ROS2 launch file in Python that starts:
  - The `turtlesim` node.
  - The `turtle_controller` node.
  - The `goal_publisher` node.

- **CMakeLists.txt**  
  Contains the build configuration for ROS2, specifying dependencies (rclcpp, geometry_msgs, turtlesim) and installing executables and launch files.

## Requirements

- ROS2 (Foxy, Galactic, Humble, or later)
- A C++14 (or later) compliant compiler
- Turtlesim package (usually installed as part of ROS2 demos)

## Installation & Build

1. **Clone the repository into your ROS2 workspace (e.g., `~/my_ros2_project/src`):**

   ```bash
   cd ~/my_ros2_project/src
   git clone <repository_url> turtlesim_path_planner
   ```
2. Build the package using colcon:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select turtlesim_path_planner
   ```
3. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Running the Project 

There are two ways to run the project.

### Using the Launch File
Run the launch file to start all nodes:

```bash
ros2 launch turtlesim_path_planner planner_launch.py
```
This will start:
- Turtlesim (the simulator)
- The Turtle Controller node
- The Goal Publisher node

### Running Nodes Individually

1. Start the turtlesim node:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
2. Run the Goal Publisher node:
   ```bash
   ros2 run turtlesim_path_planner goal_publisher
   ```
3. Run the Turtle Controller node:
   ```bash
   ros2 run turtlesim_path_planner turtle_controller
   ```

## Tuning Parameters 
You can adjust the following parameters in the source files as needed:
- **linear_velocity (default: 2.0)** – Maximum linear speed.
- **angular_gain (default: 2.0)** – Scalar to compute angular velocity from the heading error.
- **goal_threshold (default: 0.1)** – Distance threshold to consider a goal reached.
- **safe_margin** – Margin from the turtlesim boundaries used to stop forward movement when near edges.
- **min_distance_from_initial_ (default: 5.5)** – Minimum distance required between the initial pose and a newly generated goal.

## Troubleshooting

- **Turtle Not Moving**:
Ensure that the /goal topic is being published. The Turtle Controller will not move unless it receives a valid goal message.

- **Unexpected Stops**:
The turtle will stop if it is considered to have reached the goal (within goal_threshold). If it stops unexpectedly, check the goal values and thresholds.

- **Boundary Collisions**:
If the turtle collides with the window boundaries, adjust the safe_margin or implement additional logic to change its trajectory away from the border.

- **Node Crashes / No Output**:
Verify that all ROS topics are correctly published and subscribed. Check the node logs for any error messages.

## Contact 
If you have any questions or suggestions, feel free to reach out at setenay.ttc@gmail.com

  

   
   


   

   

   
   

