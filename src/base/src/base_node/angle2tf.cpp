#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <base/msg/angle.hpp>
#include <base/msg/wheels.hpp>

class Angle2TFNode : public rclcpp::Node
{
public:
    Angle2TFNode()
    : Node("angle2tf")
    {
        // Publisher erstellen
        ActualSpeed_ = this->create_publisher<base::msg::Wheels>("engine/actualSpeed", 10);

        tf_broadaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscriber für joint_states erstellen
        AngleSubs_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sensors/bodyAngle", 10, std::bind(&Angle2TFNode::AngleCallback, this, std::placeholders::_1));
    }

private:
        // Callback für die Verarbeitung der JointState-Nachricht
    void AngleCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->velocity.size() > 0)
        {
            // Erstellen und Senden der Transform-Nachricht
            tf2::Quaternion q;
            TFAngleMsg_.child_frame_id = "jointRear";
            TFAngleMsg_.header.frame_id = "jointFront";
            TFAngleMsg_.header.stamp = this->get_clock()->now();

            // Quaternion aus der Gelenkposition berechnen (hier RPY)
            q.setRPY(0, 0, msg->position[0]);

            TFAngleMsg_.transform.translation.x = 0;
            TFAngleMsg_.transform.translation.y = 0;
            TFAngleMsg_.transform.translation.z = 0;

            TFAngleMsg_.transform.rotation.x = q.x();
            TFAngleMsg_.transform.rotation.y = q.y();
            TFAngleMsg_.transform.rotation.z = q.z();
            TFAngleMsg_.transform.rotation.w = q.w();

            tf_broadaster_->sendTransform(TFAngleMsg_);

            // Zusammenbauen und Veröffentlichen der Actual Speed-Nachricht
            Wheels_actual_.header.stamp = this->get_clock()->now();

            // Geschwindigkeiten von den Gelenkzuständen übernehmen
            Wheels_actual_.front_left = msg->velocity[1];
            Wheels_actual_.front_right = msg->velocity[2];
            Wheels_actual_.rear_left = msg->velocity[3];
            Wheels_actual_.rear_right = msg->velocity[4];

            //ActualSpeed_->publish(Wheels_actual_);
        }
    }

    // Membervariablen
    rclcpp::Publisher<base::msg::Wheels>::SharedPtr ActualSpeed_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr AngleSubs_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadaster_;
    geometry_msgs::msg::TransformStamped TFAngleMsg_;
    base::msg::Wheels Wheels_actual_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Knoten erstellen und ausführen
    rclcpp::spin(std::make_shared<Angle2TFNode>());

    rclcpp::shutdown();
    return 0;
}

