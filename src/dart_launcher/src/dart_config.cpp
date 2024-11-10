#include "dart_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include <dart_msgs/msg/dart_param.hpp>

using namespace DartConfig;

void DartConfig::declareParameters(rclcpp::Node &node)
{
    node.declare_parameter("target_yaw_angle", rclcpp::ParameterType::PARAMETER_INTEGER);
    node.declare_parameter("target_yaw_angle_offset", rclcpp::ParameterType::PARAMETER_INTEGER);
    node.declare_parameter("target_fw_velocity", rclcpp::ParameterType::PARAMETER_INTEGER);
    node.declare_parameter("target_fw_velocity_offset", rclcpp::ParameterType::PARAMETER_INTEGER);
    node.declare_parameter("target_fw_velocity_ratio", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node.declare_parameter("target_yaw_launch_angle_offset", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    node.declare_parameter("target_fw_velocity_launch_offset", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
    node.declare_parameter("auto_yaw_calibration", rclcpp::ParameterType::PARAMETER_BOOL);
    node.declare_parameter("auto_fw_calibration", rclcpp::ParameterType::PARAMETER_BOOL);
    node.declare_parameter("dart_selection", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
    node.declare_parameter("target_yaw_x_axis", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node.declare_parameter("target_distance", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node.declare_parameter("target_delta_height", rclcpp::ParameterType::PARAMETER_DOUBLE);
    // set default value
    node.set_parameter(rclcpp::Parameter("target_yaw_angle", 10000));
    node.set_parameter(rclcpp::Parameter("target_yaw_angle_offset", 0));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity", 5000));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity_offset", 0));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity_ratio", 1.414));
    node.set_parameter(rclcpp::Parameter("target_yaw_launch_angle_offset", std::vector<int>{0, 0, 0, 0}));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity_launch_offset", std::vector<int>{0, 0, 0, 0}));
    node.set_parameter(rclcpp::Parameter("auto_yaw_calibration", false));
    node.set_parameter(rclcpp::Parameter("auto_fw_calibration", false));
    node.set_parameter(rclcpp::Parameter("dart_selection", std::vector<std::string>{"Default", "Default", "Default", "Default"}));
    node.set_parameter(rclcpp::Parameter("target_yaw_x_axis", 640.0));
    node.set_parameter(rclcpp::Parameter("target_distance", 0.0));
    node.set_parameter(rclcpp::Parameter("target_delta_height", 15.0));
    RCLCPP_INFO(node.get_logger(), "DartParam declared");
}

void DartConfig::loadParametersfromMsg(rclcpp::Node &node, const dart_msgs::msg::DartParam::SharedPtr msg)
{
    // 取消回调函数
    node.set_parameter(rclcpp::Parameter("target_yaw_angle", int(msg->target_yaw_angle)));
    node.set_parameter(rclcpp::Parameter("target_yaw_angle_offset", int(msg->target_yaw_angle_offset)));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity", int(msg->target_fw_velocity)));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity_offset", int(msg->target_fw_velocity_offset)));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity_ratio", double(msg->target_fw_velocity_ratio)));
    node.set_parameter(rclcpp::Parameter("target_yaw_launch_angle_offset", std::vector<int>{msg->target_yaw_launch_angle_offset[0], msg->target_yaw_launch_angle_offset[1], msg->target_yaw_launch_angle_offset[2], msg->target_yaw_launch_angle_offset[3]}));
    node.set_parameter(rclcpp::Parameter("target_fw_velocity_launch_offset", std::vector<int>{msg->target_fw_velocity_launch_offset[0], msg->target_fw_velocity_launch_offset[1], msg->target_fw_velocity_launch_offset[2], msg->target_fw_velocity_launch_offset[3]}));
    node.set_parameter(rclcpp::Parameter("auto_yaw_calibration", bool(msg->auto_yaw_calibration)));
    node.set_parameter(rclcpp::Parameter("auto_fw_calibration", bool(msg->auto_fw_calibration)));
    node.set_parameter(rclcpp::Parameter("target_yaw_x_axis", double(msg->target_yaw_x_axis)));
    // 距离和目标速度加载到参数内
    node.set_parameter(rclcpp::Parameter("target_distance", double(msg->target_distance)));
    node.set_parameter(rclcpp::Parameter("target_delta_height", double(msg->target_delta_height)));
    node.set_parameter(rclcpp::Parameter("dart_selection", std::vector<std::string>{msg->dart_selection[0], msg->dart_selection[1], msg->dart_selection[2], msg->dart_selection[3]}));
    RCLCPP_INFO(node.get_logger(), "DartParam received");
}