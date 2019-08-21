#ifndef DEMO_PLUGIN_COMPONENT__MESSAGE_WRITTER_DERIVED_H_
#define DEMO_PLUGIN_COMPONENT__MESSAGE_WRITTER_DERIVED_H_

#include "demo_plugin_component/message_writter.h"

namespace ros2_playground {

struct MessageWritterDerived : public MessageWritter
{
  MessageWritterDerived() = default;
  ~MessageWritterDerived() = default;

  inline std::string getMessageData() const override {
    return "MessageWritterDerived talking.";
  }
};

} // namespace ros2_playground

#endif // DEMO_PLUGIN_COMPONENT__MESSAGE_WRITTER_DERIVED_H_
