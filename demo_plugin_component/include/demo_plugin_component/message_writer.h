#ifndef DEMO_PLUGIN_COMPONENT__MESSAGE_WRITER_H_
#define DEMO_PLUGIN_COMPONENT__MESSAGE_WRITER_H_

#include <string>
#include <memory>

namespace ros2_playground {

struct MessageWriter
{
  using SharedPtr = std::shared_ptr<MessageWriter>;
  using UniquePtr = std::unique_ptr<MessageWriter>;

  MessageWriter() = default;
  virtual ~MessageWriter() = default;

  inline virtual std::string getMessageData() const {
    return "Default message.";
  }
};

using MessageWriterSharedPtr = MessageWriter::SharedPtr;
using MessageWriterUniquePtr = MessageWriter::UniquePtr;

} // namespace ros2_playground

#endif // DEMO_PLUGIN_COMPONENT__MESSAGE_WRITER_H_
