#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/cmdline_parser.h>
#include <std_msgs/msg/string.hpp>

#include "demo_plugin_component/message_writter.h"

using namespace std::chrono_literals;

namespace ros2_playground {

class TalkerNode : public rclcpp::Node
{
public:

  explicit TalkerNode(const rclcpp::NodeOptions & options);
  virtual ~TalkerNode() = default;

  bool init();

protected:

  std::unique_ptr<std_msgs::msg::String> getMessage() const;

  void print_usage() const;

protected:

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Plugin handling
  pluginlib::ClassLoader<MessageWritter> message_writter_loader_;
  std::shared_ptr<MessageWritter> writter_ptr_;
//  class_loader::ClassLoader::UniquePtr<MessageWritter> writter_ptr_;
};

TalkerNode::TalkerNode(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  , message_writter_loader_("demo_plugin_component", "ros2_playground::MessageWritter")
{
  auto find_command_option = [](const std::vector<std::string> & args,
                                const std::string & option) -> bool
  {
    return std::find(args.begin(), args.end(), option) != args.end();
  };

  std::vector<std::string> args = options.arguments();
  if (find_command_option(args, "-h")) {
    print_usage();
    rclcpp::shutdown();
  }

  init();
}

bool TalkerNode::init()
{
  std::string writter_name;
  this->get_parameter("writter_name", writter_name);

  try {
    writter_ptr_ = message_writter_loader_.createSharedInstance(writter_name);
  }
  catch (const pluginlib::LibraryLoadException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }
  catch (const pluginlib::CreateClassException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
  }

  if (writter_ptr_ == nullptr)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not load plugin '%s'.", writter_name.c_str());
    RCLCPP_ERROR(this->get_logger(), "Trying to instantiate default instead.");

    writter_ptr_ = std::make_shared<MessageWritter>();

//    writter_ptr_ = std::unique_ptr<MessageWritter, class_loader::ClassLoader::DeleterType<MessageWritter>>(
//          new MessageWritter,
//          std::bind(&class_loader::ClassLoader::onPluginDeletion<MessageWritter>, this, std::placeholders::_1)
//        )

    if (writter_ptr_ == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not instantiate default either.");
      return false;
    }
  }

  // Create a publisher with a custom Quality of Service profile.
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

  auto publish_message = [this]() -> void {
      auto msg = getMessage();

      if (msg == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "getMessage returned a nullptr!");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
      // Put the message into a queue to be processed by the middleware.
      // This call is non-blocking.
      pub_->publish(std::move(msg));
  };

  // Use a timer to schedule periodic message publishing.
  timer_ = this->create_wall_timer(1s, publish_message);

  return true;
}

std::unique_ptr<std_msgs::msg::String>
TalkerNode::getMessage() const {
  if (writter_ptr_ == nullptr)
  {
    RCLCPP_ERROR(this->get_logger(), "MessageWritter is a nullptr!");
    return nullptr;
  }

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = writter_ptr_->getMessageData();
  return msg;
}

void TalkerNode::print_usage() const
{
  printf("Usage for talker app:\n");
  printf("talker [-p plugin_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  // printf("-p : plugin_name : Specify which plugin to use if any.\n");
}

} // namespace ros2_playground

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_playground::TalkerNode)
