# demo_nodes_secure

This package only copies the `talker` & `listener` cpp demo from
[demo_nodes_cpp](https://github.com/ros2/demos/tree/master/demo_nodes_cpp)
and adds the [sros2](https://github.com/ros2/sros2) machinery
through [sros2_cmake](https://github.com/ros2/sros2/tree/master/sros2_cmake) helper macro

```cmake
ros2_secure_node(NODES <node_1> <node_2>...<node_n>)
```

It also adds the machinery for the `listener_best_effort` cpp demo
directly from the source package.

After building the package,

```terminal
colcon build --symlink-install --packages-select demo_nodes_secure
```

those secured demo can be run directly from the provided launch files  
*   `talker_listener_secure.launch.py`
*   `talker_listener_secure_best_effort.launch.py`
