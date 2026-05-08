#include "drives/articulated_drive.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <base/msg/wheels.hpp>
#include <stdexcept>
#include <cmath>

// Konstruktoren
// ----------------------
ArticulatedDrive::ArticulatedDrive()
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    reset();
}

ArticulatedDrive::ArticulatedDrive(
    double axesLength,
    double wheelDiameter,
    double frontLength,
    double rearLength)
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    setParam(axesLength, wheelDiameter, frontLength, rearLength);
}

ArticulatedDrive::~ArticulatedDrive() {}

// Inverse Kinematic
// ----------------------
articulatedWheelSpeed ArticulatedDrive::inverseKinematics(
    geometry_msgs::msg::Twist cmdVelMsg,
    double angle)
{
    articulatedWheelSpeed retVal;

    targetSpeed_ = cmdVelMsg.linear.x;
    targetOmega_ = cmdVelMsg.angular.z;

    if (targetSpeed_ >= 0)
    {
        angle = -angle;

        retVal.Front.leftWheel =
            (1.0 / wheelRadius_) * targetSpeed_
            - (axesLength_ / (2.0 * wheelRadius_)) * targetOmega_;

        retVal.Front.rightWheel =
            (1.0 / wheelRadius_) * targetSpeed_
            + (axesLength_ / (2.0 * wheelRadius_)) * targetOmega_;

        retVal.Rear.leftWheel =
            (std::cos(angle) / wheelRadius_
            - (axesLength_ * std::sin(angle)) / (wheelBase_ * wheelRadius_)) * targetSpeed_
            + ((wheelBase_ * std::sin(angle)) / (2.0 * wheelRadius_)
            + (axesLength_ * std::cos(angle)) / (2.0 * wheelRadius_)) * targetOmega_;

        retVal.Rear.rightWheel =
            (std::cos(angle) / wheelRadius_
            + (axesLength_ * std::sin(angle)) / (wheelBase_ * wheelRadius_)) * targetSpeed_
            + ((wheelBase_ * std::sin(angle)) / (2.0 * wheelRadius_)
            - (axesLength_ * std::cos(angle)) / (2.0 * wheelRadius_)) * targetOmega_;
    }
    else
    {
        targetOmega_ = -targetOmega_;

        retVal.Front.leftWheel =
            (std::cos(angle) / wheelRadius_
            + (axesLength_ * std::sin(angle)) / (wheelBase_ * wheelRadius_)) * targetSpeed_
            + ((wheelBase_ * std::sin(angle)) / (2.0 * wheelRadius_)
            - (axesLength_ * std::cos(angle)) / (2.0 * wheelRadius_)) * targetOmega_;

        retVal.Front.rightWheel =
            (std::cos(angle) / wheelRadius_
            - (axesLength_ * std::sin(angle)) / (wheelBase_ * wheelRadius_)) * targetSpeed_
            + ((wheelBase_ * std::sin(angle)) / (2.0 * wheelRadius_)
            + (axesLength_ * std::cos(angle)) / (2.0 * wheelRadius_)) * targetOmega_;

        retVal.Rear.leftWheel =
            (1.0 / wheelRadius_) * targetSpeed_
            + (axesLength_ / (2.0 * wheelRadius_)) * targetOmega_;

        retVal.Rear.rightWheel =
            (1.0 / wheelRadius_) * targetSpeed_
            - (axesLength_ / (2.0 * wheelRadius_)) * targetOmega_;
    }

    return retVal;
}

// Forward Kinematic
// ----------------------
geometry_msgs::msg::Pose2D ArticulatedDrive::forwardKinematics(
    articulatedWheelSpeed WheelSpeed,
    rclcpp::Time Timestamp)
{
    if (!timestamp_initialized_)
    {
        TimeStamp_ = Timestamp;
        timestamp_initialized_ = true;
        return Pose_;
    }

    double deltaTime = (Timestamp - TimeStamp_).seconds();
    TimeStamp_ = Timestamp;

    if (deltaTime < 0.0)
    {
        deltaTime = 0.0;
    }

    WheelSpeed_ = WheelSpeed.Front;

    Speed_.linear.x =
        (WheelSpeed_.leftWheel * wheelRadius_
        + WheelSpeed_.rightWheel * wheelRadius_) / 2.0;

    Speed_.angular.z =
        (WheelSpeed_.rightWheel * wheelRadius_
        - WheelSpeed_.leftWheel * wheelRadius_) / axesLength_;

    Pose_.x += Speed_.linear.x * deltaTime
        * std::cos(Pose_.theta + 0.5 * Speed_.angular.z * deltaTime);

    Pose_.y += Speed_.linear.x * deltaTime
        * std::sin(Pose_.theta + 0.5 * Speed_.angular.z * deltaTime);

    Pose_.theta += Speed_.angular.z * deltaTime;

    return Pose_;
}

// Parameter setzen
// ----------------------
void ArticulatedDrive::setParam(
    double AxesLength,
    double WheelDiameter,
    double frontLength,
    double rearLength)
{
    reset();

    axesLength_ = AxesLength;
    wheelDiameter_ = WheelDiameter;
    wheelRadius_ = wheelDiameter_ / 2.0;
    wheelCircumference_ = 2.0 * M_PI * wheelRadius_;

    frontLength_ = frontLength;
    rearLength_ = rearLength;
    wheelBase_ = frontLength_ + rearLength_;
}

void ArticulatedDrive::reset()
{
    Pose_.theta = 0.0;
    Pose_.x = 0.0;
    Pose_.y = 0.0;

    WheelSpeed_.leftWheel = 0.0;
    WheelSpeed_.rightWheel = 0.0;

    Speed_.linear.x = 0.0;
    Speed_.linear.y = 0.0;
    Speed_.linear.z = 0.0;

    Speed_.angular.x = 0.0;
    Speed_.angular.y = 0.0;
    Speed_.angular.z = 0.0;

    timestamp_initialized_ = false;
}

// Aktuelle Position vom Vorderwagen zurückgeben
// ----------------------
geometry_msgs::msg::Pose2D ArticulatedDrive::getActualPose()
{
    return Pose_;
}

// Aktuelle Geschwindigkeit zurückgeben
// ----------------------
geometry_msgs::msg::Twist ArticulatedDrive::getSpeed()
{
    return Speed_;
}
