#include <pluginlib/class_list_macros.hpp>

#include "demo_plugin_component/message_writter.h"
#include "demo_plugin_component/message_writter_derived.h"

PLUGINLIB_EXPORT_CLASS(ros2_playground::MessageWritterDerived, ros2_playground::MessageWritter)
