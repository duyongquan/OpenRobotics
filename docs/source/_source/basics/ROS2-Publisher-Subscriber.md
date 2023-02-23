# ROS2-Publisher-Subscriber

## 1 Introduction

As we understood from the lectures, nodes are the fundamental units in ROS 2 which are usually written to perform a specific task. They can be created in a few different ways such as-

1. As simple in-line code in a script,
2. As local functions, and
3. As class objects… among others

We will be using the 3rd method, though it is the more complex, so as to get better used to this concept.

We start by writing two separate simple nodes, one that includes only publisher and another that includes only a subscriber. Finally, we will write a third node that includes both within the same program and are managed through an executor.

The first step is to create a python package to house all our nodes. You can do so using the command

```shell
$ ros2 pkg create --build-type ament_python <package_name>
```

(Make sure first that ROS 2 is sourced in every new terminal)

Make sure you run this command in the *src* directory of your workspace. You can use any package name you want, but for reference in this document, we call it `wshop_nodes`.

## 2 Publisher Node

he publisher and subscriber nodes used here are in fact the [example code](https://github.com/ros2/examples/blob/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py) that ROS 2 provides.

We first present the code completely, and then discuss the interesting parts:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Inside the python package you created above, there should be another folder with the same name. Create a python file inside that folder and paste this code in. You can name the file anything you want, but for reference in this document we assign the name `minimal_publisher.py` to it.

### 1.1 Explanation

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```

`rclpy` is the *ROS 2 Client Library* that provides the API for invoking ROS 2 through Python.`Node` is the main class which will be inherited here to instantiate our own node.`std_msgs.msg` is the library for standard messages that includes the `String` message type which we use in this node. This has to be declared as a dependency in `package.xml`, which we do next.

```python
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
```

As explained above, we create a subclass of type `MinimalPublisher` using the base class `Node`.In the constructor `__init__()`, we pass the name of the node that we ish to assign to the constructer of the parent class using `super()`. The parent class `Node` takes care of actually assigning this string as a name.`self.publisher_ = self.create_publisher(String, 'topic', 10)` This line actually creates a publisher, using the message type `String` that we imported, with the name `topic` that we choose and having a queue size of `10`. Queue size is the size of the output buffer. The commands used till now are typical when creating a subscriber. What follows next is only logic that is relevant to this node, and you may implement this in any way depending on your requirements.

```python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.timer_callback)
self.i = 0
```

This creates a timer that ticks every 0.5s (2Hz), and calls the function `timer_callback` at every tick.

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

In the callback, we create an object `msg` of the type of the message we wish to publish, i.e `String`.We then populate the message with information we wish to publish. Looking at the description of the msg type using `ros2 interface show std_msgs/msg/String`, we see that it has only one field, which is `string data`. So we add a string into this field. Depending on the type of message used, you can populate it with relevant data.Once the data msg object is done, we simply publish it using the `publish()` method of the `publisher_` object.We also display this same message on the console for our verification using the `get_logger().info()` method of our `Node` class object.Publishing from within this timer callback ensures we have a consistent publishing rate of 2Hz. You could publish this in any way you want, using the proper message type and publish call.

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

In the main method, we first declare that this Python script uses the rclpy library by invoking `init()` and passing any command line arguments provided (in this case none).We instantiate an object of the class we just created. Since the contructor already spawns the timer which publishes messages, no further action is needed to setup our node.The `spin()` method ensures that all the items of work, such as callbacks, are continuously executed until a `shutdown()` is called. This is quintessential to ensure that your node actually does its job!Finally, we destroy the node and manually call shutdown.

### 1.2 Add dependencies

In the base root folder of this package, you will find the `package.xml` which is important for declaring all dependancies of the package. We will now edit this file to ensure our code runs properly.

The `description`, `maintainer` and `license` tage should be appropriately filled out. For license, use any valid open source license like `Apache License 2.0`.

The buildtool we use by default is `ament_python` and you can see that this has already been assigned when we used the `ros2 pkg create` command. Below this, add the following two lines :

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

We already know from section **1.1** what these dependancies are. We just need to declare that these two libraries need to be included during execution time. (Buildtime dependencies are not required for Python)

### 1.3 Declaring the executable

Now that we have our code written and dependencies setup, we need to tell our build system that the script we created should be treated as an executable. We do this in `setup.py`.

Here, edit the `maintainer`, `maintainer_email`, `description` and `license` fields to assign exactly the same values as you did in `package.xml`.

Next, look for the section that starts with `entry_points={`. We edit this part to declare the executable and its entry point to look like this:

```python
entry_points={
        'console_scripts': [
                'talker = wshop_nodes.minimal_publisher:main',
        ],
},
```

In this case, `talker` is the name we assign to the executable, `wshop_nodes` is the package, `minimal_publisher` is the name of the python file and `main` is the entry point to this executable (i.e. main function). Replace with the names you chose accordingly.

You can use the same prototype to declare executables in all ROS 2 python packages.

### 1.4 Setup.cfg

The final configuration file is `setup.cfg`, which, fortunately for us, is already configured properly and needs no more changes! These settings indicate to ROS 2 where the executable shall be put for discovery after building the package.



## 2 Subscriber Node

