#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "base/msg/wheel_ticks4.hpp"
#include "base/msg/wheel_velocities4.hpp"

namespace base {

class HardwareNode : public rclcpp::Node
{
public:
  HardwareNode();

private:
  using JointState = sensor_msgs::msg::JointState;

  void onWheelCmd(const base::msg::WheelVelocities4::SharedPtr msg);
  void onJointState(const JointState::SharedPtr msg);
  void publishTicks(const rclcpp::Time & stamp);
  void publishArticulation(const rclcpp::Time & stamp, double articulation_angle_rad);
  void updateSimulation();

  int resolveIndex(
    const JointState & msg,
    int configured_index,
    const std::string & configured_name,
    const std::string & field_name) const;

  std::optional<double> readValue(
    const std::vector<double> & values,
    int index,
    const std::string & field_name,
    bool warn_if_missing = true) const;

  int64_t radiansToTicks(double wheel_rotation_rad) const;

  std::string wheel_cmd_topic_{"/base/wheel_commands4"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string articulation_topic_{"/base/articulation_angle"};
  std::string joint_state_topic_{"/floribot/joint_states"};
  std::string ticks_frame_id_{"base_link"};
  bool simulation_only_{false};
  double ticks_per_rev_{131000.0};
  double publish_rate_hz_{50.0};

  int articulation_position_index_{0};
  std::string articulation_joint_name_{};

  std::array<int, 4> wheel_position_indices_{{1, 2, 3, 4}};
  std::array<int, 4> wheel_velocity_indices_{{1, 2, 3, 4}};
  std::array<std::string, 4> wheel_joint_names_{{"", "", "", ""}};

  rclcpp::Subscription<base::msg::WheelVelocities4>::SharedPtr sub_cmd_;
  rclcpp::Subscription<JointState>::SharedPtr sub_joint_state_;
  rclcpp::Publisher<base::msg::WheelTicks4>::SharedPtr pub_ticks_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_articulation_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 4> cmd_w_radps_{{0.0, 0.0, 0.0, 0.0}};
  std::array<int64_t, 4> ticks_{{0, 0, 0, 0}};
  std::array<double, 4> last_joint_positions_{{0.0, 0.0, 0.0, 0.0}};
  std::array<bool, 4> have_joint_positions_{{false, false, false, false}};
  bool warned_missing_position_{false};
  bool warned_missing_velocity_{false};

  rclcpp::Time last_update_time_;
  rclcpp::Time last_joint_state_time_;
};

}  // namespace base
