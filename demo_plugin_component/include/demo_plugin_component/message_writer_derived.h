#ifndef DEMO_PLUGIN_COMPONENT__MESSAGE_WRITER_DERIVED_H_
#define DEMO_PLUGIN_COMPONENT__MESSAGE_WRITER_DERIVED_H_

#include "demo_plugin_component/message_writer.h"

namespace ros2_playground {

struct MessageWriterDerived : public MessageWriter
{
  MessageWriterDerived() = default;
  ~MessageWriterDerived() = default;

  inline std::string getMessageData() const override {
    return "MessageWriterDerived talking.";
  }
};

} // namespace ros2_playground

#endif // DEMO_PLUGIN_COMPONENT__MESSAGE_WRITER_DERIVED_H_
