# mada

This repository contains the method MADA (Monitoring with Avoidance of Dynamic Areas) developed in [this paper](https://doi.org/10.1016/j.robot.2024.104892). The method obtains the positions from where the robot makes the observations to monitor the scenario, registers the regions where the dynamic obstacles are moving and computes the path avoiding these regions based on their occupation. The package contains the implementation of the method and the scripts to run the simulations.

## Installation

Install [Stage](http://rtv.github.io/Stage/index.html) simulator

```bash
sudo apt install ros-<ros_distro>-stage-ros
```

Clone the repository to the workspace and compile it

```bash
cd [workspace]
git clone --recursive https://github.com/yamarle/mada.git
catkin build mada
```

**NOTE**: the method was tested with Ubuntu 20.04 and ROS Noetic

## Usage

The bash file, with the configuration of the simulation can be found in the launch folder. To run the simulation:

```bash
roscd mada/launch
./start_simulation.sh
```

### Variables description
The file start_simulation.sh contains all the adjustable variables for the simulations:

*- Scenario variables:*

- *scenario*: two scenarios (*small, large*)
- *scenario_type*: refers to the number of dynamic obstacles in the scenario (*nondense, dense*)

*- Robot variables:*
- *laser_range*: the maximum distance to register monitored points of the environment
- *safety_factor*: inflation of the obstacles applied as *safety_factor* * *robot_radius*
- *maxRobotLinearVelocity*: maximum reachable linear velocity of the robot
- *maxRobotAngularVelocity*: maximum reachable angular velocity of the robot

*- Planner variables:*
- *monitoring_type*: type of monitoring method (*greedy, madp, mada*)
- *navigation_type*: navigation algorithm (*pf, dwa*)
- *occypancy_threshold*: threshold to consider the traversability of the constructed dynamic areas
	- *0*: non-traversable areas
	- (*0-1*): the area is traversable and the areas with greater values have more priority
	- *1*: areas with obstacles will be avoided if other paths through obstacle-free regions do not exist)
- *dynamic_areas_information*: manner on how the robot holds and manages the information fo the constructeed dynmic areas (*memorize_all, forget_areas, forget_all, forget_los*)

*- Obstacles variables:*
- *seed*: to generate the random motions of the obstacles (*0*: random, *>0*: selected seed)
- *maxObstLinearVelocity*: maximum reachable linear velocity of the obstacles
- *maxObstAngularVelocity*: maximum reachable angular velocity of the obstacles

