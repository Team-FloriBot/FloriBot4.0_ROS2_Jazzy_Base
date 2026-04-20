#include <rclcpp/rclcpp.hpp>
#include <base/msg/wheels.hpp>

class TestOdomPublisher : public rclcpp::Node
{
public:
    TestOdomPublisher() : Node("TestOdom")
    {
        pub_ = this->create_publisher<base::msg::Wheels>("engine/actualSpeed", 1);
        msg_.front_left = 0.9;
        msg_.front_right = 1.1;
        msg_.rear_left = 1.0;
        msg_.rear_right = 1.0;

        // Timer für das kontinuierliche Publizieren der Nachricht
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestOdomPublisher::publish_msg, this)
        );
    }

private:
    void publish_msg()
    {
        msg_.header.stamp = this->get_clock()->now();  // Verwendung von ROS2 Zeit
        pub_->publish(msg_);
    }

    rclcpp::Publisher<base::msg::Wheels>::SharedPtr pub_;
    base::msg::Wheels msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}

