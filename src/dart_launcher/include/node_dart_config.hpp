#ifndef NODE_DART_CONFIG_HPP
#define NODE_DART_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <std_srvs/srv/empty.hpp>

namespace DartConfig
{
    void declareParameters(rclcpp::Node &node);

    class NodeDartConfig : public rclcpp::Node
    {
    public:
        NodeDartConfig(const std::string &yaml_file);

    private:
        void loadParameters();
        void watchFile();
        void saveParameters();
        void syncParameters(bool spinning);
        void syncParameters(rclcpp::Parameter &param, bool spinning);

        std::string yaml_file_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
        std::filesystem::file_time_type last_write_time;

        // subscriber
        rclcpp::Subscription<info::msg::DartParam>::SharedPtr sub_dart_param_[2];

        // server for reset to default parameter yaml
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_resync_;
    };
};

#endif // PARAM_SYNC_NODE_HPP
