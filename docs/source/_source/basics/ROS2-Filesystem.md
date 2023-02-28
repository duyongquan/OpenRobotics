# ROS2-Filesystem

* author :  duyongquan
* e-mail: quandy2020@126.com

## 1  Introduction

During this tutorial, you will learn how to navigate through your ROS 2 system. In addition, you will start your first ROS 2 nodes and create your own ROS workspace for further tutorials. You can use the given links in the documentation for further information

- Lines beginning with $ are terminal commands.
  - To open a new terminal → use the shortcut ctrl+alt+t.
  - To open a new tab inside an existing terminal → use the shortcut ctrl+shift+t.
  - To kill a process in a terminal → use the shortcut crtl+c.
- Lines beginning with # indicate the syntax of the commands.

## 2 ROS File System

Before starting make sure that your system is aware of the latest ROS 2 packages:

```shell
$ sudo apt update
$ rosdep update
```

If you just installed **rosdep**, please run this commands first:

```shell
$ sudo rosdep init
```

The ROS 2 File System consists of ROS 2 packages – the smallest build part in ROS 2.

Usually, a ROS File System consists of hundreds of different packages. To navigate efficiently through your ROS system, ROS provides different management tools. The most common tools are described in the example of the ROS package turtlesim.

First of all, you have to source the ROS installation and its install workspace!

**Hint**: This has to be done for every new terminal window.

```shell
We are using **galactic** as ROS Distro

$ source /opt/ros/galactic/setup.bash
```

or

in your workspace

```shell
$ source install/setup.bash
```

## 3 Workspace

Colcon is a build tool for ROS 2. A workspace is a folder, where you can modify, build, and install packages. It is the place to create your packages and nodes or modify existing ones to fit your application. In the further tutorials, you will work in your workspace.

- Create workspace:

  ```shell
  $ cd ~
  $ mkdir -p ~/dev_ws/src
  ```

## 4  ROS packages

Two types of ROS 2 packages exist: binary packages and build-from-source packages.

ROS binary packages are in Ubuntu provided as debian packages, which can be managed via apt commands.

- Install a binary package:

  ```shell
  $ sudo apt install ros-galactic-rosbag2*
  ```

  `# sudo apt install ros-<distribution>-<package-name>`

  ros-galactic-rosbag2* means all packages which start from “ros-galactic-rosbag2”

### Locate a ROS package:

```shell
$  ros2 pkg prefix rosbag2
# ros2 pkg prefix <package_name>
```

### List executables:

```shell
ros2 pkg executables action_tutorials_py
# ros2 pkg executables <package_name>
```

Build-from-source packages can be divided into packages provided by ROS 2 and your developments. Both have to be placed into the src folder of a ROS 2 workspace.

- Install an available build-from-source package:

  ***Hint: Ensure you’re still in the dev_ws/src directory before you clone.***

  ```shell
  $ cd ~/dev_ws/src
  $ git clone https://github.com/ros/ros_tutorials.git -b galactic-devel
  ```

  `# git clone -b <branch> <address>`

  Notice the “Branch” drop-down list to the left above the directories list. When you clone this repo, add the -b argument followed by the branch that corresponds with your ROS 2 distro.

  To see the packages inside ros_tutorials, enter the command:

  ```shell
  $ ls ros_tutorials
  ```

  You will find you have four packages: `roscpp_tutorials rospy_tutorials ros_tutorials turtlesim`

  Only **turtlesim** is ROS 2 package

- Resolve dependencies:

  Before building the workspace, you need to resolve package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait because of missing dependencies.

  ***Hint: Ensure you’re in the workspace root (~/dev_ws) directory before you run rosdep.***

  If it is your first time to run `rosdep`, you need to run `rosdep init` first.

  ```shell
  $ cd ..
  $ rosdep install --from-paths src --ignore-src -r -y
  ```

  This command magically installs all the packages that the packages in your workspace depend upon but are missing on your computer. http://wiki.ros.org/rosdep

- Build the workspace with colcon

  ***Hint: Don’t forget to source the ROS installation before build and make sure you are in the root of workspace(~/dev_ws).***

  ```shell
  $ source /opt/ros/galactic/setup.bash
  
  $ colcon build --symlink-install
  ```

- Source the overlay When you build this workspace, your main ROS 2 environment is the “underlay”. Now you can source overlay “on the top of” “underlay”.

  ***Hint***: If you open a new terminal, set up ROS 2 environment first.

  ```shell
  you can start a new terminal window by
  ctl + alt +t
  ```

  ```shell
  $ cd dev_ws
  $ source install/setup.bash
  ```

## 5 ROS nodes

A ROS node is an executable in the ROS environment. A node is always a part of a ROS package. How to start ROS nodes and how to display runtime information is explained in the example of the turtlesim package.

- Start a ROS node:

  Hint: You have to start each process in a separate terminal. Don’t forget to source the ROS installation.

  ```shell
  $ ros2 run turtlesim turtlesim_node
  ```

  `# ros2 run <package_name> <executable_node_name>`

  A new window should pop up, displaying a turtle.

- Start another node to control the turtle:

  ```shell
  $ ros2 run turtlesim turtle_teleop_key
  ```

  `# ros2 run <package_name> <executable_node_name>`

  You can control the turtle with the arrow keys of the keyboard, if the current terminal running the turtle_teleop_node is selected.

The two nodes turtlesim_node and turtle_teleop_key are currently active on your ROS 2 system. Since systems that are more complex will consist of many nodes, ROS offers introspection tools to display information about active nodes.

- Display all active nodes:

  ```shell
  $ ros2 node list
  ```

- Display information about a node:

  ```shell
  $ ros2 node info /turtlesim
  ```

  `# ros2 node info <active_node_name>`

The given information comprises topics that are subscribed and published by this node.