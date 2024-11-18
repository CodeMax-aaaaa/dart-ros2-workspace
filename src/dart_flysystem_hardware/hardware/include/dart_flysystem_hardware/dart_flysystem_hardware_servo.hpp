/**
 * @file dart_flysystem.hpp
 * @brief 飞镖飞行系统硬件控制库
 */

#ifndef DART_FLYSYSTEM_HARDWARE_SERVO_HPP
#define DART_FLYSYSTEM_HARDWARE_SERVO_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dart_flysystem_hardware/linux_pwm.hpp"
#include <unordered_map>
#include <vector>

namespace dart_flysystem_hardware
{
    class DartFlySystemHardwareServo : public hardware_interface::SystemInterface
    {
        RCLCPP_SHARED_PTR_DEFINITIONS(DartFlySystemHardwareServo)

    private:
        // PWM类
        std::unordered_map<std::string, std::shared_ptr<linuxPWM::LinuxPwm>> pwm_map_;
        std::unordered_map<std::string, double> angle_map_;
        std::unordered_map<std::string, double> min_angle_map_;
        std::unordered_map<std::string, double> max_angle_map_;
        bool debug_;
        double offset_angle_;

    public:
        DartFlySystemHardwareServo() = default;
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    };
} // namespace dart_flysystem

#endif // DART_FLYSYSTEM_HARDWARE_SERVO_HPP