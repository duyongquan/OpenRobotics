# ROS2-Parameters

* author :  yipeng
* e-mail: evenap96@qq.com

## 1 ROS2参数概述

### 1.1参数定义

ROS2官方对参数的定义是：

**A parameter is a configuration value of a node. You can think of parameters as node settings.**

**参数是节点的一个配置值，可以认为参数是一个节点的设置**

### 1.2参数的组成

ROS2参数是由键值对组成，如

```yaml
name: "zhangsan"
age: 5
```

其中ROS2支持的值的数据类型有（定义在ros2/rcl_interfaces）：

- bool 和bool[]
- int64 和int64[]
- float64 和float64[]
- string 和string[]
- byte[]

## 2 参数使用示例

### 2.1 声明并使用参数

```c++
// 声明参数名，并为其赋初值
this->declare_parameter("param_double", 0.0f);
this->declare_parameter("param_int", 0);
this->declare_parameter("param_float", 0.);
this->declare_parameter("param_str", "");

// 读取参数的值
param_double_ = this->get_parameter("param_double").as_double();
param_int_ = this->get_parameter("param_int").as_int();
param_float_ = this->get_parameter("param_float").as_double();
param_str_ = this->get_parameter("param_str").as_string();
```

### 2.2 使用默认参数值

按照如下方式编译并运行代码，上述参数值为默认值

 ``` bash
 colcon build --packages-up-to params
 source install/setup.bash 
 ros2 run params tutorial.params.yaml_config_test
 ```

### 2.3 使用launch文件设置参数

若launch文件中设置了参数值，启动时会覆盖参数的初始值

```python
 Node(
     package='params',
     executable='tutorial.params.yaml_config_test',
     parameters=[
         {'param_double': 200.02},
         {'param_int': 400},
         {'param_float': 500.5},
         {'param_str': 'Hello String'},
     ])
```

运行launch文件

```bash
ros2 launch params  tutorials_params_demo1_yaml_conmfig_test.launch.py 
```

### 2.4 使用ymal文件设置参数

启动时也可以读取ymal文件来覆盖参数的初始值，使用ymal配置参数需使用固定格式。其中，minimal_paser为Node名，ros__parameters为固定值，其余的为键值对，示例如下

```ymal
/minimal_paser:
  ros__parameters:
    param_double: 17.7
    param_int_: 17
    param_float: 17.0
    param_str: "HelloYmalParams"

```

加载ymal参数并启动节点

``` bash
ros2 run  params tutorial.params.yaml_config_test --ros-args --params-file ./parameters.yaml
```

## 3 有关参数的命令

```bash
ros2 param list
ros2 param get /minimal_paser /param_double
ros2 param set /minimal_paser param_double 10.0
ros2 param describe /minimal_paser /param_double
ros2 param dump /minimal_paser
ros2 param load  /minimal_paser ./parameters.yaml

```





