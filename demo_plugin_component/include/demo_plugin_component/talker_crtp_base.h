#ifndef DEMO_PLUGIN_COMPONENT__TALKER_CRTP_BASE_H_
#define DEMO_PLUGIN_COMPONENT__TALKER_CRTP_BASE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_playground {

// A CRTP base class to factor away all of the common code
template <class Derived>
class TalkerCrtpBase : public rclcpp::Node
{
  inline const Derived& derived() const & noexcept {
      return *static_cast< const Derived* >(this);
  }

public:

  explicit TalkerCrtpBase(const rclcpp::NodeOptions & options)
    : Node("talker_crtp", options)
  {
    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    pub_ = this->create_publisher<std_msgs::msg::String>("crtp_chatter", qos);

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
  }

  virtual ~TalkerCrtpBase() = default;

protected:

  std::unique_ptr<std_msgs::msg::String> getMessage() const {
    return derived().getMessageImpl();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace ros2_playground

#endif // DEMO_PLUGIN_COMPONENT__TALKER_CRTP_BASE_H_
