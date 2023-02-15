<center> <font color=green size=10> ROS2 Tutorials</font> </center>

# 1 Introduction

The documents are written in `Markdown` language and built using [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?) documentation generator.

# 2 Usage

## 2.1 Install dependencies

```shell
cd ./ros2_tutorials/docs
python3 -m pip install -r requirements.txt
```

## 2.2 build the html

```shell
cd ./ros2_tutorials/docs
mkdir build && cmake ..
```

# 3 Contents

* ROS2 Basics
  * topic
  * parameters
  * service
  * action
  * timer
  * lifecycle
  * composition
  * executors
  * rviz
  * gazebo
* ROS2 SLAM
  * Cartographer SLAM
  * SLAM Toolbox
* ROS2 Navigation
  * TF2
  * costmap
  * gridmap
  * behavior tree
  * path planner
  * path controller

