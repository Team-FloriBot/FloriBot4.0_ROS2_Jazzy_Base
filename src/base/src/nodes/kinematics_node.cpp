#include "base/kinematics_node.h"

#include <cmath>
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;

namespace base {

static inline bool isZeroStamp(const builtin_interfaces::msg::Time & t)
{
  return (t.sec == 0) && (t.nanosec == 0);
}

KinematicsNode::KinematicsNode()
: Node("kinematics_node")
{
  params_cb_ = add_on_set_parameters_callback(
    std::bind(&KinematicsNode::onParams, this, std::placeholders::_1));

  declare_parameter("track_width", track_width_);
  declare_parameter("wheel_radius", wheel_radius_);
  declare_parameter("front_joint_distance", front_joint_distance_);
  declare_parameter("rear_joint_distance", rear_joint_distance_);
  declare_parameter("ticks_per_rev", ticks_per_rev_);
  declare_parameter("unwrap_modulo", unwrap_modulo_);
  declare_parameter("modulo_ticks", modulo_ticks_);
  declare_parameter("publish_tf", publish_tf_);
  declare_parameter("odom_source_axle", odom_source_axle_);

  declare_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter("articulation_topic", articulation_topic_);
  declare_parameter("wheel_cmd_topic", wheel_cmd_topic_);
  declare_parameter("wheel_ticks_topic", wheel_ticks_topic_);
  declare_parameter("odom_topic", odom_topic_);
  declare_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("base_frame_id", base_frame_id_);

  track_width_ = get_parameter("track_width").as_double();
  wheel_radius_ = get_parameter("wheel_radius").as_double();
  front_joint_distance_ = get_parameter("front_joint_distance").as_double();
  rear_joint_distance_ = get_parameter("rear_joint_distance").as_double();
  ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();
  unwrap_modulo_ = get_parameter("unwrap_modulo").as_bool();
  modulo_ticks_ = get_parameter("modulo_ticks").as_int();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  odom_source_axle_ = get_parameter("odom_source_axle").as_string();

  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  articulation_topic_ = get_parameter("articulation_topic").as_string();
  wheel_cmd_topic_ = get_parameter("wheel_cmd_topic").as_string();
  wheel_ticks_topic_ = get_parameter("wheel_ticks_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();

  rebuildCalculator();

  if (modulo_ticks_ <= 0) {
    modulo_ticks_ = static_cast<int>(std::llround(ticks_per_rev_));
  }

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10, std::bind(&KinematicsNode::cmdVelCallback, this, _1));

  sub_articulation_ = create_subscription<std_msgs::msg::Float64>(
    articulation_topic_, 10, std::bind(&KinematicsNode::articulationCallback, this, _1));

  sub_ticks_ = create_subscription<base::msg::WheelTicks4>(
    wheel_ticks_topic_, 50, std::bind(&KinematicsNode::wheelTicksCallback, this, _1));

  pub_wheel_cmd_ = create_publisher<base::msg::WheelVelocities4>(wheel_cmd_topic_, 10);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  last_odom_time_ = now();

  RCLCPP_INFO(
    get_logger(),
    "kinematics_node started | cmd=%s articulation=%s wheel_cmd=%s ticks=%s odom=%s",
    cmd_vel_topic_.c_str(), articulation_topic_.c_str(), wheel_cmd_topic_.c_str(),
    wheel_ticks_topic_.c_str(), odom_topic_.c_str());
}

void KinematicsNode::rebuildCalculator()
{
  if (track_width_ <= 0.0 || wheel_radius_ <= 0.0 ||
      front_joint_distance_ <= 0.0 || rear_joint_distance_ <= 0.0 ||
      ticks_per_rev_ <= 0.0)
  {
    throw std::runtime_error("Invalid kinematics parameter set");
  }

  kinematics_ = std::make_unique<KinematicsCalculator>(
    track_width_, wheel_radius_, front_joint_distance_, rear_joint_distance_);
}

rcl_interfaces::msg::SetParametersResult
KinematicsNode::onParams(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool rebuild = false;
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "track_width") { track_width_ = p.as_double(); rebuild = true; }
    else if (name == "wheel_radius") { wheel_radius_ = p.as_double(); rebuild = true; }
    else if (name == "front_joint_distance") { front_joint_distance_ = p.as_double(); rebuild = true; }
    else if (name == "rear_joint_distance") { rear_joint_distance_ = p.as_double(); rebuild = true; }
    else if (name == "ticks_per_rev") { ticks_per_rev_ = p.as_double(); }
    else if (name == "unwrap_modulo") { unwrap_modulo_ = p.as_bool(); }
    else if (name == "modulo_ticks") { modulo_ticks_ = p.as_int(); }
    else if (name == "publish_tf") { publish_tf_ = p.as_bool(); }
    else if (name == "odom_source_axle") { odom_source_axle_ = p.as_string(); }
  }

  try {
    if (rebuild) {
      rebuildCalculator();
    }
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void KinematicsNode::articulationCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  articulation_angle_rad_ = msg->data;
}

void KinematicsNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto ws = kinematics_->calculateWheelSpeeds(
    msg->linear.x, msg->angular.z, articulation_angle_rad_);

  base::msg::WheelVelocities4 out;
  out.fl = ws.fl;
  out.fr = ws.fr;
  out.rl = ws.rl;
  out.rr = ws.rr;
  pub_wheel_cmd_->publish(out);
}

void KinematicsNode::wheelTicksCallback(const base::msg::WheelTicks4::SharedPtr msg)
{
  const rclcpp::Time stamp = isZeroStamp(msg->header.stamp) ? now() : rclcpp::Time(msg->header.stamp);

  const int64_t fl = msg->fl_ticks;
  const int64_t fr = msg->fr_ticks;
  const int64_t rl = msg->rl_ticks;
  const int64_t rr = msg->rr_ticks;

  if (!have_prev_ticks_) {
    have_prev_ticks_ = true;
    prev_fl_ = fl; prev_fr_ = fr; prev_rl_ = rl; prev_rr_ = rr;
    last_odom_time_ = stamp;
    publishOdom(stamp, 0.0, 0.0);
    return;
  }

  const double dt = (stamp - last_odom_time_).seconds();
  if (!(dt > 0.0)) {
    publishOdom(stamp, 0.0, 0.0);
    return;
  }

  const int64_t dfl = unwrap_modulo_ ? deltaModulo(fl, prev_fl_, modulo_ticks_) : (fl - prev_fl_);
  const int64_t dfr = unwrap_modulo_ ? deltaModulo(fr, prev_fr_, modulo_ticks_) : (fr - prev_fr_);
  const int64_t drl = unwrap_modulo_ ? deltaModulo(rl, prev_rl_, modulo_ticks_) : (rl - prev_rl_);
  const int64_t drr = unwrap_modulo_ ? deltaModulo(rr, prev_rr_, modulo_ticks_) : (rr - prev_rr_);

  prev_fl_ = fl; prev_fr_ = fr; prev_rl_ = rl; prev_rr_ = rr;
  last_odom_time_ = stamp;

  const double k = (2.0 * M_PI) / ticks_per_rev_;

  const double ds_fl = wheel_radius_ * static_cast<double>(dfl) * k;
  const double ds_fr = wheel_radius_ * static_cast<double>(dfr) * k;
  const double ds_rl = wheel_radius_ * static_cast<double>(drl) * k;
  const double ds_rr = wheel_radius_ * static_cast<double>(drr) * k;

  double ds_left = 0.0;
  double ds_right = 0.0;
  if (odom_source_axle_ == "rear") {
    ds_left = ds_rl;
    ds_right = ds_rr;
  } else {
    ds_left = ds_fl;
    ds_right = ds_fr;
  }

  const double ds = 0.5 * (ds_left + ds_right);
  const double dtheta = (ds_right - ds_left) / track_width_;

  const double theta_mid = theta_ + 0.5 * dtheta;
  x_ += ds * std::cos(theta_mid);
  y_ += ds * std::sin(theta_mid);
  theta_ = normalizeAngle(theta_ + dtheta);

  publishOdom(stamp, ds / dt, dtheta / dt);
}

void KinematicsNode::publishOdom(const rclcpp::Time & stamp, double vx, double wz)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = wz;
  pub_odom_->publish(odom);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = base_frame_id_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
}

int64_t KinematicsNode::deltaModulo(int64_t cur, int64_t last, int64_t mod)
{
  if (mod <= 0) {
    return cur - last;
  }

  int64_t d = cur - last;
  const int64_t half = mod / 2;
  if (d > half) {
    d -= mod;
  } else if (d < -half) {
    d += mod;
  }
  return d;
}

double KinematicsNode::normalizeAngle(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}

}  // namespace base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<base::KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
