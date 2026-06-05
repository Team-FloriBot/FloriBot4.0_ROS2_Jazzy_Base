#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "cmd_vel_selector/srv/select_source.hpp"

using namespace std::chrono_literals;

class CmdVelSelectorNode : public rclcpp::Node
{
public:
  using Twist = geometry_msgs::msg::Twist;
  using SelectSource = cmd_vel_selector::srv::SelectSource;
  using Trigger = std_srvs::srv::Trigger;

  CmdVelSelectorNode()
  : Node("cmd_vel_selector"),
    source_names_{
      "webteleop",
      "tasks"
    }
  {
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic",
      "/cmd_vel");

    active_source_ = this->declare_parameter<std::string>(
      "initial_source",
      "none");

    if (!isValidSource(active_source_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ungueltige initial_source '%s'. Setze aktive Quelle auf 'none'.",
        active_source_.c_str());

      active_source_ = "none";
    }

    output_publisher_ = this->create_publisher<Twist>(
      output_topic_,
      rclcpp::QoS(10));

    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.reliable();
    status_qos.transient_local();

    active_source_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/cmd_vel_selector/active_source",
      status_qos);

    createSourceSubscriptions();

    select_source_service_ = this->create_service<SelectSource>(
      "/cmd_vel_selector/select_source",
      std::bind(
        &CmdVelSelectorNode::selectSourceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stop_service_ = this->create_service<Trigger>(
      "/cmd_vel_selector/stop",
      std::bind(
        &CmdVelSelectorNode::stopCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    selected_at_ = this->now();
    last_active_message_time_ = this->now();

    publishStop();
    publishActiveSource();

    RCLCPP_INFO(
      this->get_logger(),
      "cmd_vel_selector gestartet. Ausgang: %s, aktive Quelle: %s",
      output_topic_.c_str(),
      active_source_.c_str());
  }

private:
  void createSourceSubscriptions()
  {
    for (const auto & source_name : source_names_) {
      const std::string parameter_name = "topics." + source_name;
      const std::string default_topic = "/cmd_vel/" + source_name;

      const std::string topic_name = this->declare_parameter<std::string>(
        parameter_name,
        default_topic);

      source_topics_[source_name] = topic_name;

      auto subscription = this->create_subscription<Twist>(
        topic_name,
        rclcpp::QoS(10),
        [this, source_name](const Twist::SharedPtr message) {
          this->cmdVelCallback(source_name, *message);
        });

      source_subscriptions_.push_back(subscription);

      RCLCPP_INFO(
        this->get_logger(),
        "Quelle '%s' abonniert Topic '%s'.",
        source_name.c_str(),
        topic_name.c_str());
    }
  }

  void cmdVelCallback(
    const std::string & source_name,
    const Twist & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (source_name != active_source_) {
      return;
    }

    last_active_message_time_ = this->now();
    has_received_message_from_active_source_ = true;
    output_publisher_->publish(message);
  }

  void selectSourceCallback(
    const std::shared_ptr<SelectSource::Request> request,
    std::shared_ptr<SelectSource::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!isValidSource(request->source)) {
      response->success = false;
      response->active_source = active_source_;
      response->message =
        "Ungueltige Quelle. Erlaubt sind: " + validSourceList();

      RCLCPP_WARN(
        this->get_logger(),
        "Ungueltige Quellenanforderung: '%s'.",
        request->source.c_str());

      return;
    }

    if (request->source == active_source_) {
      response->success = true;
      response->active_source = active_source_;
      response->message =
        "Quelle '" + active_source_ + "' ist bereits aktiv.";

      return;
    }

    const std::string previous_source = active_source_;

    active_source_ = request->source;
    selected_at_ = this->now();
    has_received_message_from_active_source_ = false;

    // Sicherheitsstopp beim Umschalten:
    // Die neue Quelle muss nach der Auswahl aktiv einen neuen Befehl senden.
    publishStop();
    publishActiveSource();

    response->success = true;
    response->active_source = active_source_;
    response->message =
      "Quelle von '" + previous_source +
      "' auf '" + active_source_ + "' umgeschaltet.";

    RCLCPP_INFO(
      this->get_logger(),
      "Aktive cmd_vel-Quelle gewechselt: '%s' -> '%s'.",
      previous_source.c_str(),
      active_source_.c_str());
  }

  void stopCallback(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;

    std::lock_guard<std::mutex> lock(mutex_);

    active_source_ = "none";
    selected_at_ = this->now();
    has_received_message_from_active_source_ = false;

    publishStop();
    publishActiveSource();

    response->success = true;
    response->message =
      "Fahrbefehl gestoppt. Aktive Quelle ist 'none'.";

    RCLCPP_WARN(
      this->get_logger(),
      "Stop-Service ausgefuehrt. Aktive Quelle ist jetzt 'none'.");
  }

  void publishStop()
  {
    Twist stop_message;
    output_publisher_->publish(stop_message);
  }

  void publishActiveSource()
  {
    std_msgs::msg::String message;
    message.data = active_source_;
    active_source_publisher_->publish(message);
  }

  bool isValidSource(const std::string & source_name) const
  {
    if (source_name == "none") {
      return true;
    }

    for (const auto & valid_source : source_names_) {
      if (source_name == valid_source) {
        return true;
      }
    }

    return false;
  }

  std::string validSourceList() const
  {
    std::ostringstream stream;
    stream << "none";

    for (const auto & source_name : source_names_) {
      stream << ", " << source_name;
    }

    return stream.str();
  }

  std::vector<std::string> source_names_;
  std::map<std::string, std::string> source_topics_;

  std::string output_topic_;
  std::string active_source_;

  bool has_received_message_from_active_source_{false};

  rclcpp::Time selected_at_;
  rclcpp::Time last_active_message_time_;

  std::mutex mutex_;

  rclcpp::Publisher<Twist>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_source_publisher_;

  std::vector<rclcpp::Subscription<Twist>::SharedPtr> source_subscriptions_;

  rclcpp::Service<SelectSource>::SharedPtr select_source_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CmdVelSelectorNode>());

  rclcpp::shutdown();

  return 0;
}
