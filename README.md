# Robotics-Maze-Solver

Authors: ...




## How to run
Open one of the provided CoppeliaSim scenes provided in the `scenes` folder:
- `scenes/scene-sensors_small.ttt` for the small maze and proximity sensors navigation
- `scenes/scene-sensors_big.ttt` for the big maze and proximity sensors navigation
- `scenes/scene-camera_small.ttt` for the small maze and camera navigation
- `scenes/scene-camera_big.ttt` for the big maze and camera navigation

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


