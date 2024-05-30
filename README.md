# Robotics-Maze-Solver

Authors: Alessandro Gobbetti, Lovnesh Bhardwaj, Lin Xintan

## Description
This project is based on Micromouse competitions, an event held since the 1970s where small robotic mice compete to solve a $16\times16$ maze. During the execution, the robot needs to keep track of where it is, discover the placement of the walls to map the whole maze, and detect when it has reached its goal. In this project, we use a Thymio robot in CoppeliaSim software to simulate the environment. To solve the maze, the robot needs to employ fundamental elements of robot navigation, including mapping, planning, and localization. A very simple way to transverse the maze is simply following the left/right wall. However, using this simple technique, the robot may be stuck in a loop.
Additionally, the robot needs to optimize its final path. There are a few common search algorithms that can be used, such as Bellman-Ford, Dijkstra's algorithm, and A* search algorithm. These algorithms ensure to find the optimal path every time, but they require complete knowledge of the maze and the maze scanning will take a lot of time. Instead, we decided to use an optimistic approach to reach the goal and take advantage of the fact that the robot had to go back to the starting square to collect more information on the maze. We thus use the flood-fill algorithm to plan the path.
This project can be split into three main parts:
- Solve the maze: given a certain knowledge of the maze, find the best path to the goal.
- Move in the maze: given the next cell's direction and distance, reach it.
- Detect the surroundings: use sensors to detect walls.

This project is for the Robotics Course offered at Università della Svizzera Italiana (USI), Lugano.

## How to run
Open one of the provided CoppeliaSim scenes provided in the `resource/scenes` folder:
- `resource/scenes/scene-sensors_small.ttt` for the small maze and proximity sensors navigation
- `resource/scenes/scene-sensors_big.ttt` for the big maze and proximity sensors navigation
- `resource/scenes/scene-camera_small.ttt` for the small maze and camera navigation
- `resource/scenes/scene-camera_big.ttt` for the big maze and camera navigation

To run the code, move the `Robotics-Maze-Solver` package in your ROS workspace and build it:
```bash
colcon build
```

Then open two terminals, cd into your workspace and source the workspace in both of them:
```bash
source install/setup.bash
```

In the first terminal create a bridge for the Thymio robot:
```bash
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0
```

In the second terminal run the main node. There are two different nodes, one for the proximity sensors and one for the camera:
```bash
ros2 launch maze_solver proximity.launch.xml
```
or
```bash
ros2 launch maze_solver camera.launch.xml
```

Also, with the additional parameter `maze:=example_big` or `maze:=example_small` you can specify which maze to use. The default is `example_small`. A complete example is:
```bash
ros2 launch maze_solver camera.launch.xml maze:=example_big
```

## Processed Image Output
To look at what the processed image that helps the robot in aligning to the center of the cell, open another terminal, and run:
```bash
rqt
```
Then click on Plugins > Visualization > Image View and enter the following topic:
```bash
/camera/image_processed
```
This will yield the processed image that the camera is seeing. Please note that a Lua script had to be added to the Camera Sensor to get the processed image. This can be found in the file `CameraSensorChildScript.lua` in the `resource` directory .

## Example Videos
In the 'resource/videos' folder, you can find some example videos of the robot navigating the maze using the proximity sensors and the camera.
