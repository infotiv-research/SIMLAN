#include "bt_failsafe/stop_robot.hpp"
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>  

using std::placeholders::_1;

namespace bt_failsafe
{

StopRobotCondition::StopRobotCondition(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config), contact_detected_(false)
{
    if (!getInput("robot_ns", robot_ns_))
    {
        throw std::runtime_error("StopRobotCondition missing required input [robot_ns]");
    }

    auto bb = config.blackboard;
    node_ = bb->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
        throw std::runtime_error("StopRobotCondition: ROS node not found on blackboard!");
    }

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.transient_local();
    qos.reliable();

    // Subscribe to ArUco marker lost
    aruco_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/aruco_marker_seen", qos,
        std::bind(&StopRobotCondition::aruco_callback, this, _1));

    // Subscribe to Gazebo contacts
    collision_sub_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "/" + robot_ns_ + "/contact", rclcpp::SensorDataQoS(),
        std::bind(&StopRobotCondition::collision_callback, this, _1));
}

void StopRobotCondition::aruco_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Clear old list
    marker_seen_.clear();

    // Split the comma-separated list into individual robots
    std::stringstream ss(msg->data);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        if (!item.empty())
        {
            marker_seen_.push_back(item);
        }
    }
}

void StopRobotCondition::collision_callback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
{
    contact_detected_ = false;

    for (const auto &contact : msg->contacts)  // iterate over each contact
    {
        for (const auto &wrench : contact.wrenches) // iterate over wrenches in the contact
        {
            auto &f = wrench.body_1_wrench.force;
            if (std::abs(f.x) > FORCE_THRESHOLD ||
                std::abs(f.y) > FORCE_THRESHOLD ||
                std::abs(f.z) > FORCE_THRESHOLD)
            {
                contact_detected_ = true;
                break;
            }
        }
    }
}

BT::NodeStatus StopRobotCondition::tick()
{
    rclcpp::spin_some(node_);

    // Check out of bounds
    if (std::find(marker_seen_.begin(), marker_seen_.end(), robot_ns_) == marker_seen_.end())
    {
        RCLCPP_WARN(node_->get_logger(),
            "%s is out of bounds", robot_ns_.c_str());
        return BT::NodeStatus::FAILURE; // triggers CancelControl
    }

    // Check collision
    if (contact_detected_)
    {
        RCLCPP_WARN(node_->get_logger(),
            "%s : collision detected!", robot_ns_.c_str());
        contact_detected_ = false;
        return BT::NodeStatus::FAILURE; // triggers CancelControl
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_failsafe
