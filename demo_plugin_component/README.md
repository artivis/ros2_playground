# demo_plugin_component

A demo package to illustrate the use of `pluginlib` and components in `ROS 2`.

### Usage

Like a plain node,
```terminal
$ ros2 run demo_plugin_component talker_node __params:=src/ros2_playground/demo_plugin_component/cfg/params.yaml
```

As a component in a component container,
```terminal
ros2 run rclcpp_components component_container
```
```terminal
ros2 component load /ComponentManager demo_plugin_component ros2_playground::TalkerNode -p writer_name:='ros2_playground::MessageWriterDerived'
```

Same as above, from a launch file,
```terminal
ros2 launch demo_plugin_component talker.launch.py
```

Open a `topic echo` to monitor the node output,
```terminal
ros2 topic echo /chatter
```

#### CRTP demo

```terminal
ros2 run rclcpp_components component_container
```

```terminal
ros2 component load /ComponentManager demo_plugin_component ros2_playground::TalkerCrtpDerivedA
ros2 component load /ComponentManager demo_plugin_component ros2_playground::TalkerCrtpDerivedB
```
