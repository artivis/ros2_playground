# demo_plugin_component

A demo package to illustrate the use of `pluginlib` and components in `ROS 2`.

## Plugins vs. Components

> ***Long story short, components are plugins.***

Short story long ? Is that a thing ?

Well plugins and components are indeed essentially the same thing.  
Down the road, both are built into respective shared libraries,
neither have a `main` function,
they both are to be loaded and use by a third party.

So what are the actual differences ?

To put it simply, a component is a plugin which derives from a `ROS 2` node.

This assertion is backed by the fact the both rely on the
[`class_loader` package](https://github.com/ros/class_loader),
a ROS-independent library for dynamic class introspection
and loading from runtime libraries.
Their respective internal plugin-related plumbing (factory, registration, path finding,
loading etc.) is managed by `class_loader`.
Both offer a macro-based helper for registration,
and both macros resolves
to `class_loader`'s `CLASS_LOADER_REGISTER_CLASS` macro.  
Yeah they immediately resolve to it, I mean, they don't even try to hide it.
On the other hand, why would they ?

For a traditional plugin, the base class can be anything, user defined or not.
The only constraint is that the derived class has to be default constructible
(therefore, by the law of C++, the base class too).
In the case of components,
the plugin class commonly derives from the `rclcpp::Node` class,
but it is not required.
Indeed, the requirements for a class to be exported as a component are,
* Have a constructor that takes a single argument that is a `rclcpp::NodeOptions` instance.
* Have a method of of the signature:
  * `rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface(void)`

which includes `rclcpp::Node` of course, but also e.g. `rclcpp_lifecycle::LifecycleNode`.

It means that a component can inherit from any class meeting those requirements or
even implement them itself and skip inheritance altogether.  
This last point is another important difference between plugins and components.  
Indeed, traditionally plugins must declare their base classes when registering
to `class_loader` factory scheme.
Components' registration on the other hand was developed
relying on `void` (smart) pointer-based type-erasure. Thus allowing to
declare a simple wrapper class as a common base class to all components.

In most cases, components inherit from `rclcpp::Node`
as it is the easiest way to fulfill the above requirements.
Therefore, we'll assume that a component **is** a `ROS 2` node,
with everything it involves,
possibly parameters, listeners/publishers, services, action, tutti quanti.

We can therefore make an important distinction here,
components are plugin **with a ROS 2 interface**.

## Show me some code

### Usage

Like a plain node,
```terminal
$ ros2 run demo_plugin_component talker_node __params:=src/ros2_playground/demo_plugin_component/cfg/params.yaml
```

As a component,
```terminal
ros2 run rclcpp_components component_container
```
```terminal
ros2 component load /ComponentManager demo_plugin_component ros2_playground::TalkerNode -p writter_name:='ros2_playground::MessageWritterDerived'
```
