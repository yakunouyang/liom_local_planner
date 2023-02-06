# liom_local_planner ROS Package

![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)
![stability-experimental](https://img.shields.io/badge/stability-experimental-orange.svg)

A ROS package for motion planning for car-like robots, providing a fast and reliable local planner for autonomous vehicles. The package is designed for use in robotics applications, with a focus on navigation and control for ground vehicles.

## Citing

If you use `liom_local_planner` in your work, please consider citing our related papers to acknowledge the research and efforts behind this package:

 - B. Li et al., "Optimization-Based Trajectory Planning for Autonomous Parking With Irregularly Placed Obstacles: A Lightweight Iterative Framework," in IEEE Transactions on Intelligent Transportation Systems, vol. 23, no. 8, pp. 11970-11981, Aug. 2022, doi: [10.1109/TITS.2021.3109011](https://ieeexplore.ieee.org/abstract/document/9531561).
 - B. Li, Y. Ouyang, L. Li and Y. Zhang, "Autonomous Driving on Curvy Roads Without Reliance on Frenet Frame: A Cartesian-Based Trajectory Planning Method," in IEEE Transactions on Intelligent Transportation Systems, vol. 23, no. 9, pp. 15729-15741, Sept. 2022, doi: [10.1109/TITS.2022.3145389](https://ieeexplore.ieee.org/abstract/document/9703250).


## Features

 - Fast and efficient motion planning for car-like robots
 - Robust and reliable operation in a variety of scenarios
 - Easy integration into existing robotic systems

## Requirements

 - ROS Kinetic or later
 - Ubuntu 16.04 or later

## Installation

1. Create a new workspace:

```shell
$ mkdir -p ~/liom_ws/src
$ cd ~/liom_ws/src
$ catkin_init_workspace
```

2. Clone the package into the workspace:

```shell
$ git clone https://github.com/yakunouyang/liom_local_planner.git
```

3. Install dependencies:
```shell
rosdep install liom_local_planner
```

3. Build the workspace:

```shell
$ cd ~/liom_ws
$ catkin_make
```

## Usage

1. Launch the simulation based on [Stage Simulator](http://wiki.ros.org/stage):

    ```shell
    $ roslaunch liom_local_planner demo_stage.launch
    ```

    Publish a goal pose to the `/move_base_simple/goal` topic.


2. Launch test node:

    ```shell
   roslaunch liom_local_planner demo_test.launch
    ```
    
    feel free to drag the interactive obstacles.


## Documentation

For more information on using the liom_local_planner package, please refer to the [official documentation](https://example.com).

## Contributing

We welcome contributions to the liom_local_planner package! If you are interested in contributing, please see the [contributing guide](https://example.com) for more information.

## License

The liom_local_planner package is released under the BSD 3-Clause license. See [LICENSE](https://github.com/yakunouyang/liom_local_planner/blob/master/LICENSE) for more information.




