#ifndef NODE_DART_LOGGER_DOG_HPP
#define NODE_DART_LOGGER_DOG_HPP
#include <rclcpp/rclcpp.hpp>
#include <info/msg/dart_launcher_status.hpp>
#include <info/msg/judge.hpp>
#include <info/msg/dart_param.hpp>

class NodeLoggerDog : public rclcpp::Node
{
private:
    rclcpp::Subscription<info::msg::DartLauncherStatus>::SharedPtr dart_launcher_status_sub_;
    rclcpp::Subscription<info::msg::Judge>::SharedPtr judge_sub_;
    rclcpp::Subscription<info::msg::DartParam>::SharedPtr dart_param_present_sub_;
    rclcpp::Subscription<info::msg::DartParam>::SharedPtr dart_param_sub_;
    
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;

public:
    NodeLoggerDog();

private:
    void dart_launcher_status_callback(const info::msg::DartLauncherStatus::SharedPtr msg);
    void judge_callback(const info::msg::Judge::SharedPtr msg);
    void dart_param_callback(const info::msg::DartParam::SharedPtr msg);
    void on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

    bool check_changes(const info::msg::Judge *last_msg, const info::msg::Judge::SharedPtr msg);
    bool check_changes(const info::msg::DartLauncherStatus *last_msg, const info::msg::DartLauncherStatus::SharedPtr msg);
    bool check_changes(const info::msg::DartParam *last_msg, const info::msg::DartParam::SharedPtr msg);

    bool is_node_running(const std::string &node_name);
    void check_nodes();
    void restart_service();

    std::string exec(const char *cmd);

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> nodes_to_watch_;
    std::string service_name_;
    bool enable;
};
#endif // NODE_DART_LOGGER_DOG_HPP