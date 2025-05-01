/**
 * @file node_dart_param_gateway.hpp
 * @brief NodeDartParamGateway 类头文件
 */
#ifndef NODE_DART_PARAM_GATEWAY_HPP
#define NODE_DART_PARAM_GATEWAY_HPP
// ROS2 Lifecycle Node
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// Dart_msg
#include <dart_msgs/msg/dart_param.hpp>
#include <dart_msgs/msg/dart_launcher_status.hpp>

// ROS2 Service
#include <std_srvs/srv/empty.hpp>

// Json
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class NodeDartParamGateway : public rclcpp_lifecycle::LifecycleNode
{
public:
    NodeDartParamGateway();
    ~NodeDartParamGateway();
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

private:
    // ROS2 Lifecycle Node
    rclcpp::Node::SharedPtr node_;

    // Publisher
    rclcpp::Publisher<dart_msgs::msg::DartStatus>::SharedPtr dart_param_pub_;

    // Dart_param
    dart_msgs::msg::DartParam dart_param_;
};

#endif