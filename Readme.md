## Pure pursuit controller
Basic implementation of the pure pursuit controller. For running this example you need to install the jackal simulator following the instructions from [clearpath robotics website](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html).

It launches a visual interface that allows to click for setting the waypoints.

For running the simulation, source your catkin_ws and do:
```
roslaunch pure_persuit jackal_world_fy.launch
```

For running the actual controller cd into the /src folder and do:
```
python pure_persuit.py
```