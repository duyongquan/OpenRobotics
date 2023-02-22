# ROS2-Introduction

## 1 搭建环境

* ubuntu20
* ROS2 galactic

新建工程目录

```shell
mkdir -p ros2_ws/src
cd ros2_ws/src
```

工程目录

```
└── ros2_ws
    └── src
        ├── grid_map
        ├── navigation2
        └── ros2_tutorials
```

## 2 源码依赖

* navigation2

  ```shell
  git clone -b galactic https://github.com/ros-planning/navigation2.git
  ```

* grid_map

  ```
  git clone -b ros2 https://github.com/ANYbotics/grid_map.git
  ```

* ros2_tutorials

  ```shell
  git clone -b https://github.com/duyongquan/ros2_tutorials
  ```

## 3 编译

* 编译topic

  ```
  colcon build --packages-up-to topic
  ```

* 编译nav2_demos

  ```
  colcon build --packages-up-to path_planning 
  ```

* 编译path_planning

  ```
  colcon build --packages-up-to path_planning 
  ```

## 4 注意事项

* grid_map编译问题

  * 问题1 : 如果cv_bridge/cv_bridge.hpp出现错误

    ```c++
    // OpenCV
    #include <cv_bridge/cv_bridge.hpp> 改为 #include <cv_bridge/cv_bridge.h>
    ```

  * 问题2: 文件GridMapRosConverter.cpp中，出现to_yaml错误

    ```c++
    // RCLCPP_INFO(
    //   rclcpp::get_logger("fromOccupancyGrid"),
    //   "Orientation of occupancy grid: \n%s",
    //   geometry_msgs::msg::to_yaml(occupancyGrid.info.origin.orientation).c_str());
    注释掉此代码
    ```

    