# OCS2 Mobile Manipulator

The `ocs2_mobile_manipulator` package supports various robotic arms and wheel-based mobile manipulators. The system model is determined by parsing the URDF and the task file.

Over here, we specify the steps involved in creating the URDF used for the examples.

* [Franka Panda](#franka-panda)
* [Kinova Jaco2](#kinova-jaco2)
* [Willow Garage PR2](#willow-garage-pr2)
* [Clearpath Ridgeback with UR-5 arm](#clearpath-ridgeback-with-ur-5)

## Franka Panda

* In the `src` directory of your catkin workspace, clone the official repository of the [Franka Panda](https://www.franka.de/):

```bash
git clone git@github.com:frankaemika/franka_ros.git
```

* Build the necessary packages and source the workspace:

```bash
catkin build franka_description ocs2_robotic_assets

source devel/setup.bash
```

* Convert the xacro file to urdf format:

```bash
rosrun xacro xacro -o $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/franka/urdf/panda.urdf $(rospack find franka_description)/robots/panda_arm.urdf.xacro hand:=true
```

* Copy all meshes from `franka_description` to `ocs2_robotic_assets/resources` directory:

```bash
cp -r $(rospack find franka_description)/meshes $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/franka/meshes
```

* Replace the meshes locations in the robot's urdf

```bash
sed -i 's+franka_description+ocs2_robotic_assets/resources/mobile_manipulator/franka+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/franka/urdf/panda.urdf
```

* Add a dummy link "root" to the URDF (KDL prefers the root of the tree to have an empty-link):

```xml
  ...
  <!-- Root link -->
  <link name="root"/>
  <joint name="root_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="root"/>
    <child link="panda_link0"/>
  </joint>
  <!-- Robot Arm -->
  ...
```

## Kinova Jaco2

* In the `src` directory of your catkin workspace, clone the official repository of the [Kinova Jaco2](https://assistive.kinovarobotics.com/product/jaco-robotic-arm):

```bash
git clone git@github.com:Kinovarobotics/kinova-ros.git
```

* Build the necessary packages and source the workspace:

```bash
catkin build kinova_description ocs2_robotic_assets

source devel/setup.bash
```

* Convert the xacro file to urdf format:

```bash
# For Kinova Jaco2 (7-DOF)
rosrun xacro xacro -o $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/kinova/urdf/j2n7s300.urdf $(rospack find kinova_description)/urdf/j2n7s300_standalone.xacro
# For Kinova Jaco2 (6-DOF)
rosrun xacro xacro -o $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/kinova/urdf/j2n6s300.urdf $(rospack find kinova_description)/urdf/j2n6s300_standalone.xacro
```

* Copy all meshes from `franka_description` to `ocs2_robotic_assets/resources` directory:

```bash
cp -r $(rospack find kinova_description)/meshes $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/kinova/meshes
```

* Replace the meshes locations in the robot's urdf

```bash
# For Kinova Jaco2 (7-DOF)
sed -i 's+kinova_description+ocs2_robotic_assets/resources/mobile_manipulator/kinova+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/kinova/urdf/j2n7s300.urdf
# For Kinova Jaco2 (6-DOF)
sed -i 's+kinova_description+ocs2_robotic_assets/resources/mobile_manipulator/kinova+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/kinova/urdf/j2n6s300.urdf
```

* In addition to above, we make the following changes to the URDF Wto make it more readable:

    * Replace all arm joint types from "continuous" to "revolute" with high joint limits
    * (optional) Removed all `gazebo` tags, i.e.: `<gazebo>`, `<transmission>`
    * (optional) Removed the following links from the chain: `world`

## Willow Garage PR2

* In the `src` directory of your catkin workspace, clone the official repository of the [PR2 robot](https://robots.ieee.org/robots/pr2/):

```bash
git clone git@github.com:PR2/pr2_common.git
```

* Build the necessary packages and source the workspace:

```bash
catkin build pr2_description ocs2_robotic_assets

source devel/setup.bash
```

* Convert the xacro file to urdf format:

```bash
rosrun xacro xacro -o $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/pr2/urdf/pr2.urdf $(rospack find franka_description)/robots/pr2.urdf.xacro
```

* Copy all meshes from `pr2_description` to `ocs2_robotic_assets/resources` directory:

```bash
cp -r $(rospack find pr2_description)/meshes $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/pr2/meshes
cp -r $(rospack find pr2_description)/materials $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/pr2/materials
```

* Replace the meshes locations in the robot's urdf

```bash
sed -i 's+pr2_description+ocs2_robotic_assets/resources/mobile_manipulator/pr2+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/pr2/urdf/pr2.urdf
```

* In addition to above, we make the following changes to the URDF Wto make it more readable:

    * Replace all arm joint types from "continuous" to "revolute" with high joint limits
    * Changed all "screw" joints (i.e. `l_gripper_motor_screw_joint`, `r_gripper_motor_screw_joint`, `torso_lift_motor_screw_link`) to fixed joints
    * (optional) Removed all `gazebo` tags, i.e.: `<gazebo>`, `<transmission>`

## Clearpath Ridgeback with UR-5

* In the `src` directory of your catkin workspace, clone the official repository for the [Ridgeback](https://github.com/ridgeback/ridgeback_manipulation):

```bash
git clone https://github.com/ridgeback/ridgeback_manipulation.git --recursive
git clone https://github.com/ridgeback/ridgeback.git
```

* In the `src` directory of your catkin workspace, clone the packages for the arm to use (in this case [UR5](https://www.universal-robots.com/)):

```bash
vcs import . < ./ridgeback_manipulation/ur.rosinstall
```

* Resolve all the ROS dependencies (top of your catkin workspace):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

* Build the necessary packages and source the workspace:

```bash
catkin build ur_description ridgeback_description ridgeback_ur_description ocs2_robotic_assets

source devel/setup.bash
```

* Set the arm to mount on Clearpath Ridgeback base:

```
source $(rospack find ridgeback_ur_description)/scripts/setup_ridgeback_ur5_envar
```

* Convert the xacro file to urdf format:

```bash
rosrun xacro xacro -o $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf $(rospack find ridgeback_description)/urdf/ridgeback.urdf.xacro
```

* Copy all meshes from `pr2_description` to `ocs2_robotic_assets/resources` directory:

```bash
mkdir -p $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/meshes/base
mkdir -p $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/meshes/ur5
cp -r $(rospack find ridgeback_description)/meshes/* $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/meshes/base
cp -r $(rospack find ur_description)/meshes/ur5/* $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/meshes/ur5
```

* Replace the meshes locations in the robot's urdf

```bash
sed -i 's+ridgeback_description/meshes+ocs2_robotic_assets/resources/mobile_manipulator/ridgeback_ur5/meshes/base+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf
sed -i 's+ur_description/meshes/ur5+ocs2_robotic_assets/resources/mobile_manipulator/ridgeback_ur5/meshes/ur5+g' $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf
```

* In addition to above, we make the following changes to the URDF Wto make it more readable:

    * Replace all wheel joint types from "continuous" to "fixed"
    * Replace `front_rocker` joint type from "revolute" to "fixed"
    * (optional) Removed all `gazebo` tags, i.e.: `<gazebo>`, `<transmission>`
