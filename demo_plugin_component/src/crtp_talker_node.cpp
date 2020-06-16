#include "demo_plugin_component/talker_crtp_base.h"

namespace ros2_playground {

struct TalkerCrtpDerivedA
  : public TalkerCrtpBase<TalkerCrtpDerivedA>
{
  explicit TalkerCrtpDerivedA(const rclcpp::NodeOptions & options)
    : TalkerCrtpBase<TalkerCrtpDerivedA>(options) {}

  // getMessage 'overide'
  inline std::unique_ptr<std_msgs::msg::String> getMessageImpl() const {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "TalkerCrtpDerivedA talking.";
    return msg;
  }
};

struct TalkerCrtpDerivedB
  : public TalkerCrtpBase<TalkerCrtpDerivedB>
{
  using Base = TalkerCrtpBase<TalkerCrtpDerivedB>;
  // Use Base constructor
  using Base::Base;

  // getMessage 'overide'
  inline std::unique_ptr<std_msgs::msg::String> getMessageImpl() const {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "TalkerCrtpDerivedB talking.";
    return msg;
  }
};

} // namespace ros2_playground

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_playground::TalkerCrtpDerivedA)
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_playground::TalkerCrtpDerivedB)
