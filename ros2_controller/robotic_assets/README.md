# OCS2 Robotic Assets

This package provides a collection of assets for several commonly used robots. The assets are meant to serve as examples on integrating various different robots with [OCS2 Toolbox](https://github.com/leggedrobotics/ocs2).

__Note:__ For reproducibility, the steps involved in creating the mobile-manipulator assets are described
in [`docs/MobileManipulatorUrdfAssets.md`](docs/MobileManipulatorUrdfAssets.md).

## Assets

### Toy robots

| Robot | Description | Reference | License |
|-------|-------------|-----------|---------|
| [__Cart-pole__](resources/cartpole/) | An ideal 1-D cart with vertical bar attached to it.  | -  | - |
| [__Double integrator__](resources/double_integrator) |   An ideal 1-D cart with dynamics model.  | - | - |

### Complex robots

| Robot | Description | Reference | License |
|-------|-------------|-----------|---------|
| [__ANYmal-C__](resources/anymal_c/) | The 12-DoF torque-controlled quadrupedal from ANYbotics AG  |  [Link](https://ieeexplore.ieee.org/document/7758092)  |   [BSD-3](https://github.com/ANYbotics/anymal_c_simple_description/blob/master/LICENSE) |
| [__Ballbot__](resources/ballbot) |   The ball-balancing mobile base (2D-XY + RPY).  | [Link](https://arxiv.org/abs/1902.10415) |  -  |
[__Franka Panda__](resources/mobile_manipulator/franka/) |  A 7-DoF arm with parallel-jaw gripper. | [Link](https://www.franka.de/) | [Apache 2.0](http://wiki.ros.org/franka_description) |
[__Kinova Jaco2__](resources/mobile_manipulator/kinova/) |  The 6-DoF and 7-DoF arms with three fingers gripper. | [Link](https://www.franka.de/) | [BSD-3](https://github.com/Kinovarobotics/kinova-ros/blob/master/LICENSE) |
[__Mabi-Mobile__](resources/mobile_manipulator/mabi_mobile/) |  A 6-DoF arm with differential-drive base under holonomic constraint. | [Link](https://www.research-collection.ethz.ch/handle/20.500.11850/439902) | - |
[__Ridgeback-UR5__](resources/mobile_manipulator/ridgeback_ur5/) |  A 6-DoF UR-5 arm on ClearPath Ridgeback base. | - | [Apache 2.0](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot) |
| [__PR2__](resources/mobile_manipulator/pr2) |  The PR2 robot (omni-directional base).  | [Link](https://ieeexplore.ieee.org/document/5980058) | [BSD-3](http://wiki.ros.org/pr2_description) |
| [__Quadrotor__](resources/quadrotor) |  An ideal quadrotor with its dynamics model.  | - | - |
