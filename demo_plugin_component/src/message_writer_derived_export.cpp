#include <pluginlib/class_list_macros.hpp>

#include "demo_plugin_component/message_writer.h"
#include "demo_plugin_component/message_writer_derived.h"

PLUGINLIB_EXPORT_CLASS(ros2_playground::MessageWriterDerived, ros2_playground::MessageWriter)
