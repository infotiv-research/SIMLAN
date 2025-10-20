#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

namespace bt_failsafe
{

class StopRobotCondition : public BT::ConditionNode
{
public:
    StopRobotCondition(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("robot_ns") };
    }

    BT::NodeStatus tick() override;

private:
    void aruco_callback(const std_msgs::msg::String::SharedPtr msg);
    void collision_callback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr aruco_sub_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr collision_sub_;
    std::vector<std::string> marker_seen_;
    std::string robot_ns_;
    bool contact_detected_;
    const double FORCE_THRESHOLD = 1.0;
};

}  // namespace bt_failsafe
