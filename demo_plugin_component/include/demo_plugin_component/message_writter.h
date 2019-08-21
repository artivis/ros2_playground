#ifndef DEMO_PLUGIN_COMPONENT__MESSAGE_WRITTER_H_
#define DEMO_PLUGIN_COMPONENT__MESSAGE_WRITTER_H_

#include <string>
#include <memory>

namespace ros2_playground {

struct MessageWritter
{
  using SharedPtr = std::shared_ptr<MessageWritter>;
  using UniquePtr = std::unique_ptr<MessageWritter>;

  MessageWritter() = default;
  virtual ~MessageWritter() = default;

  inline virtual std::string getMessageData() const {
    return "Default message.";
  }
};

using MessageWritterSharedPtr = MessageWritter::SharedPtr;
using MessageWritterUniquePtr = MessageWritter::UniquePtr;

} // namespace ros2_playground

#endif // DEMO_PLUGIN_COMPONENT__MESSAGE_WRITTER_H_
