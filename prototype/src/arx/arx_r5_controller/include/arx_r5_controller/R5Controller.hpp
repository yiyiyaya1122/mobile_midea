#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include "arx_r5_src/interfaces/InterfacesThread.hpp"

#include "robot_msgs/msg/robot_cmd.hpp"
#include "robot_msgs/msg/robot_status.hpp"
#include "robot_msgs/msg/pos_cmd.hpp"

namespace arx::r5 {
class R5Controller : public rclcpp::Node {
 public:
  R5Controller();
  void cleanup() {
    interfaces_ptr_.reset();
  }

  void CmdCallback(const robot_msgs::msg::RobotCmd::SharedPtr msg);
  void PubState();

  void VrCmdCallback(const robot_msgs::msg::PosCmd::SharedPtr msg);
  void VrPubState();

  void FollowCmdCallback(const robot_msgs::msg::RobotStatus::SharedPtr msg);

 private:
  std::shared_ptr<InterfacesThread> interfaces_ptr_;
  enum class CatchControlMode {
    kPosition,
    kTorque
  } catch_control_mode_;

  // 通常 & remote从机模式下
  rclcpp::Publisher<robot_msgs::msg::RobotStatus>::SharedPtr joint_state_publisher_;
  // vr模式下
  rclcpp::Publisher<robot_msgs::msg::PosCmd>::SharedPtr vr_joint_state_publisher_;

  // 通常模式下
  rclcpp::Subscription<robot_msgs::msg::RobotCmd>::SharedPtr joint_state_subscriber_;
  // vr模式下
  rclcpp::Subscription<robot_msgs::msg::PosCmd>::SharedPtr vr_joint_state_subscriber_;
  // remote从机模式下
  rclcpp::Subscription<robot_msgs::msg::RobotStatus>::SharedPtr follow_joint_state_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}