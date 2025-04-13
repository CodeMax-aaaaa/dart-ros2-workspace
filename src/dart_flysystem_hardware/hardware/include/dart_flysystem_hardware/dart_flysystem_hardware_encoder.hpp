/**
 * @file dart_flysystem_hardware_encoder.hpp
 * @brief 飞镖系统编码器硬件接口（AS5600 I2C 版本）
 */

#ifndef DART_FLYSYSTEM_HARDWARE_ENCODER_HPP
#define DART_FLYSYSTEM_HARDWARE_ENCODER_HPP

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <thread>

namespace dart_flysystem_hardware
{
    class DartFlySystemHardwareEncoder : public hardware_interface::SensorInterface
    {
        RCLCPP_SHARED_PTR_DEFINITIONS(DartFlySystemHardwareEncoder)

    private:
        // I2C 配置参数
        std::string i2c_device_ = "/dev/i2c-4"; // 默认设备路径
        std::string dartI2CEN_path = "/sys/class/leds/dart-i2c-en/brightness"; // I2C Local EN Pin
        int i2c_address_ = 0x36;                // AS5600 默认地址
        int i2c_fd_ = -1;                       // 设备文件描述符

        // 编码器数据
        double encoder_angle_ = 0.0;                     // 角度值（弧度）
        const double angle_scale_ = (2 * M_PI) / 4096.0; // 12-bit 分辨率
        double encoder_velocity_ = 0.0;                  // 角速度值（弧度/秒）
        int encoder_offset_ = 0;                    // 零点偏移值（弧度）

        // I2C 初始化
        bool initI2C();
        int readAngleRegister(uint8_t reg);

        // I2C 读取线程
        bool thread_running = false;
        std::thread i2c_read_thread_;
        void i2cReadThread();

    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        ~DartFlySystemHardwareEncoder() noexcept override = default;
    };
} // namespace dart_flysystem_hardware

#endif // DART_FLYSYSTEM_HARDWARE_ENCODER_HPP