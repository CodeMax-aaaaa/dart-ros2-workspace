/**
 * @file dart_flysystem_hardware_imu.hpp
 * @brief 飞镖飞行系统IMU Hardware Interface
 */

#ifndef DART_FLYSYSTEM_HARDWARE_IMU_HPP
#define DART_FLYSYSTEM_HARDWARE_IMU_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "wit_c_sdk.h"
#include <unordered_map>
#include <vector>
#include <termios.h>
#include <thread>

#define SERIAL_BUFFER_SIZE 1024

namespace dart_flysystem_hardware
{
    class DartFlySystemHardwareIMU : public hardware_interface::SensorInterface
    {
        RCLCPP_SHARED_PTR_DEFINITIONS(DartFlySystemHardwareIMU)

    private:
        // Serial Port
        std::string serial_file_;
        static int serial_fd_;
        static volatile char new_data_;
        uint8_t serial_buffer_[SERIAL_BUFFER_SIZE];

        // IMU Data
        double imu_data_raw_linear_acceleration_[3];
        double imu_data_raw_angular_velocity_[3];
        double imu_data_raw_orientation_[4];

        // Thread for Read Serial Port
        std::thread serial_thread_;
        bool serial_thread_running_;

        // Serial Daemon Thread
        speed_t getBaudrate(int baudrate);
        void serialDaemonThread();
        static void serialWrite(uint8_t *data, uint32_t len);
        static void updateNotifier(uint32_t uiReg, uint32_t uiRegNum);

    public:
        DartFlySystemHardwareIMU() = default;
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    };
} // namespace dart_flysystem

#endif // DART_FLYSYSTEM_HARDWARE_IMU_HPP