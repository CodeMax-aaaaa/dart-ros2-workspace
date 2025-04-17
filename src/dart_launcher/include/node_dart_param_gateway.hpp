/**
 * @file node_dart_param_gateway.hpp
 * @brief NodeDartParamGateway 类头文件
 */
#ifndef NODE_DART_PARAM_GATEWAY_HPP
#define NODE_DART_PARAM_GATEWAY_HPP
// ROS2 Lifecycle Node
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// ROS2 Service
#include <std_srvs/srv/empty.hpp>

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
};

#endif