## Rules

The arena is 3.7m x 3.7m.

The carriers are 360mm square with ArUco ID tags on each of the four sides.

The dropoff zone is the rightmost 0.5m.

At the start of a run, there will be five robots distributed at random orientations with their centres in the dropoff zone. There will be ten carriers with ramdom orientation distributed in the lefthand side of the arena.

Obstacles will be 300mm x 300mm blocks placed between the robots and the carriers.












To run the various scenarios, do the following, editing controller.launch.py to launch your controllers nodes.



### Unordered
```
ros2 launch dots_example_controller run_5_controller.launch.py use_gzweb:=true use_scorer:=true
```

### Ordered
```
ros2 launch dots_example_controller run_5_controller.launch.py use_gzweb:=true use_scorer:=true carrier_order:='[104,101,102,100,103,107,109,108,105,106]'
```

### Unordered, obstacles
```
ros2 launch dots_example_controller run_5_controller.launch.py use_gzweb:=true use_scorer:=true world_file:=install/dots_sim/share/dots_sim/worlds/arena_obstacle.world
```

### Ordered, obstacles
```
ros2 launch dots_example_controller run_5_controller.launch.py use_gzweb:=true use_scorer:=true world_file:=install/dots_sim/share/dots_sim/worlds/arena_obstacle.world carrier_order:='[104,101,102,100,103,107,109,108,105,106]'
```

