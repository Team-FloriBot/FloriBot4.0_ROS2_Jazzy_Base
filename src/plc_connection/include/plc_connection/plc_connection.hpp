#ifndef PLC_CONNECTION_OBJ_H
#define PLC_CONNECTION_OBJ_H

#include "network/udp/udp_socket.hpp"
#include <linux/if_link.h>
#include <ifaddrs.h>
#include <math.h>
#include <base/msg/wheels.hpp>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <base/msg/angle.hpp>
#include <plc_connection/srv/get_count.hpp>

#include <chrono>

struct FromPLC
{
    uint32_t MessageID;
    uint32_t Mode;
    float Speed[4];
    uint32_t Angle;
    float Voltage;
    uint32_t HomingError;
    uint32_t SpeedError[4];
    uint32_t ResetError[4];
    float Angle_rad;
};

struct ToPLC
{
    uint32_t MessageID;
    uint32_t Mode;
    float Speed[4];
    float Accelleration;
    float Jerk;
    float Dummy[4];
};

struct PLC_Data
{
    FromPLC From;
    ToPLC To;
};

struct TargetDevice
{
    OwnUDP::Address IP;
    uint64_t LastID;
    rclcpp::Time LastMsgTime;
    bool ComOk;
};

//node class
class PlcConnectionNode : public rclcpp::Node
{
public:
    //Constructor
    
    PlcConnectionNode(OwnUDP::Address* OwnIP);
    PlcConnectionNode(OwnUDP::Address* OwnIP, OwnUDP::Address* TargetIP);
    PlcConnectionNode();
    ~PlcConnectionNode();
    
private:
    //Initialize Subscriber/Publisher/Socket
    void Subscribe();
    void CreatePublisher();
    void InitializeSocket();
    
    //Read ROS-Param
    void ReadParams();

    //Timer Function
    void SendRecv();
    
    //Send/receive Data
    void SendData();
    bool ReadData();
    void PublishData();
    
    //Callback for ServiceCount Callback
    bool GetCountService(
        const std::shared_ptr<plc_connection::srv::GetCount::Request> req, 
        std::shared_ptr<plc_connection::srv::GetCount::Response> res);
    
    //Callback for Speed subscriber 
    void SpeedCallback(const base::msg::Wheels::SharedPtr msg);
    
    //Callback for Mode subscriber
    void ModeCallback(const std_msgs::msg::UInt32::SharedPtr msg);

    //Callback for Torque subscriber
    void TorqueCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    //Callback for Accelaration subscriber
    void AccelerationCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    //Subscriber
    rclcpp::Subscription<base::msg::Wheels>::SharedPtr SpeedSubscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr ModeSubscriber_;
        
    //Publisher
    rclcpp::Publisher<base::msg::Wheels>::SharedPtr SpeedPublisher_;
    rclcpp::Publisher<base::msg::Angle>::SharedPtr AnglePublisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> TFBroadcaster_;
    
    //Service
    rclcpp::Service<plc_connection::srv::GetCount>::SharedPtr CountServer_;
    rclcpp::TimerBase::SharedPtr SendRecvTimer_;
    
    //Rosparams
    std::string strTargetIP_, strOwnIP_;
    uint TargetPort_, OwnPort_, ReceiveTimeoutSec_, ReceiveTimeoutUsec_;
    double PLCTimeout_;
    
    //Duration for Connection Timeout
    rclcpp::Duration ConnectionTimeout_;
    
    //Params for Angle
    int zeroCount_;
    float countPerRotation_;
    double readWritePeriod_;
    
    //Connection ok status
    bool ComOk_;
    unsigned int seq_;
    
    //UDP-Socket
    OwnUDP::UDPSocket PLC_Socket_;
    PLC_Data Data_;
    
    //PLC Data to Exchange
    TargetDevice Target_;
};

void ntohPLC(PLC_Data* Host, PLC_Data* Network);
void htonPLC(PLC_Data* Network, PLC_Data* Host);

#endif