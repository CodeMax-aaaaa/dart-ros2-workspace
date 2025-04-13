/**
 * @file dart_flysystem_hardware_encoder.cpp
 * @brief 飞镖飞行系统Hardware Encoder Interface编码器硬件接口
 */
#include "dart_flysystem_hardware/dart_flysystem_hardware_encoder.hpp"
#include "lifecycle_msgs/msg/state.hpp"

// i2c-dev
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

namespace dart_flysystem_hardware
{
    static int i2c_read_bytes(int fd, uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len)
    {
        uint8_t outbuf[1];
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[2];

        outbuf[0] = reg_addr;
        messages[0].addr = slave_addr;
        messages[0].flags = 0;
        messages[0].len = sizeof(outbuf);
        messages[0].buf = outbuf;

        /* The data will get returned in this structure */
        messages[1].addr = slave_addr;
        messages[1].flags = I2C_M_RD /* | I2C_M_NOSTART*/;
        messages[1].len = len;
        messages[1].buf = values;

        /* Send the request to the kernel and get the result back */
        packets.msgs = messages;
        packets.nmsgs = 2;
        if (ioctl(fd, I2C_RDWR, &packets) < 0)
        {
            printf("Error: Unable to send data");
            return -1;
        }

        return 0;
    }

    // initI2C
    bool DartFlySystemHardwareEncoder::initI2C()
    {
        // 打开 I2C 设备
        i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
        if (i2c_fd_ < 0)
        {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Failed to open I2C device: " << i2c_device_);
            return false;
        }

        return true;
    }

    // 读取线程
    void DartFlySystemHardwareEncoder::i2cReadThread()
    {
        // 缓冲区
        uint8_t buffer[2];
        int error_count = 0;

        RCLCPP_INFO(
            this->get_logger(),
            "I2C Read Thread Start");

        // 时间戳
        auto last_time = std::chrono::steady_clock::now();
        double encoder_angle_last_ = 0.0;

        while (true)
        {
            // 尝试首次读取角度寄存器
            if (i2c_read_bytes(i2c_fd_, i2c_address_, 0x0E, buffer, 2) == 0)
            {
                // 计算角度值
                int angle = (buffer[0] << 8) | buffer[1];
                angle += encoder_offset_;
                angle %= 4096;
                encoder_angle_ = (angle * angle_scale_);
                encoder_angle_last_ = encoder_angle_;
                thread_running = true;
                error_count = 0;
                break;
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to read encoder angle");
                error_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (error_count > 10)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to read encoder angle, closing I2C device");
                close(i2c_fd_);
                thread_running = false;
                return;
            }
        }
        while (thread_running)
        {
            // 500Hz 采样
            if (i2c_read_bytes(i2c_fd_, i2c_address_, 0x0E, buffer, 2) == 0)
            {
                // 计算角度值
                int angle = (buffer[0] << 8) | buffer[1];
                angle += encoder_offset_;
                angle %= 4096;
                encoder_angle_ = (angle * angle_scale_);

                // 计算角速度
                auto current_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);

                // 读取ms值，计算角速度rad/s
                // 过零点的情况下，不予计算
                if (encoder_angle_ - encoder_angle_last_ > M_PI)
                {
                    encoder_angle_last_ += 2 * M_PI;
                }
                else if (encoder_angle_ - encoder_angle_last_ < -M_PI)
                {
                    encoder_angle_last_ -= 2 * M_PI;
                }
                encoder_velocity_ = (encoder_angle_ - encoder_angle_last_) / duration.count() * 1000.0;

                last_time = current_time;
                encoder_angle_last_ = encoder_angle_;

                error_count = 0;
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to read encoder angle");
                error_count++;

                // 重试 10 次
                if (error_count > 10)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to read encoder angle, closing I2C device");

                    close(i2c_fd_);

                    initI2C();
                    error_count = 0;

                    // 等待100ms重启I2C
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            // 等待2ms
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        RCLCPP_INFO(
            this->get_logger(),
            "I2C Read Thread Exit");
    }

    // on_init
    hardware_interface::CallbackReturn DartFlySystemHardwareEncoder::on_init(const hardware_interface::HardwareInfo &info)
    {
        // 从硬件信息中获取 I2C 设备路径和地址，如果没有则使用默认值
        if (info.hardware_parameters.find("i2c_device") != info.hardware_parameters.end())
        {
            i2c_device_ = info.hardware_parameters.at("i2c_device");
        }
        if (info.hardware_parameters.find("i2c_address") != info.hardware_parameters.end())
        {
            i2c_address_ = std::stoi(info.hardware_parameters.at("i2c_address"), 0, 16);
        }
        if (info.hardware_parameters.find("encoder_raw_offset") != info.hardware_parameters.end())
        {
            encoder_offset_ = std::stoi(info.hardware_parameters.at("encoder_raw_offset"));
        }
        if (info.hardware_parameters.find("dart_i2c_en_path") != info.hardware_parameters.end())
        {
            dartI2CEN_path = info.hardware_parameters.at("dart_i2c_en_path");
        }

        // 输出 I2C 配置信息
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "I2C Device: " << i2c_device_ << ", I2C Address: 0x" << std::hex << i2c_address_
                           << ", Encoder Offset: " << encoder_offset_ << ", I2C Local EN Path: " << dartI2CEN_path);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // on_configure
    hardware_interface::CallbackReturn DartFlySystemHardwareEncoder::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        // 启动 I2C Local 域隔离，向/sys/class/led/dart-i2c-en/brightness写入0
        if (system(("sudo chmod 666 " + dartI2CEN_path).c_str()) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set permission for I2C Local EN Pin: %s", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }
        // 初始化 I2C
        if (!initI2C())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // on_activate
    hardware_interface::CallbackReturn DartFlySystemHardwareEncoder::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        int fd = open(dartI2CEN_path.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C Local EN Pin: %s", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (write(fd, "0", 1) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to I2C Local EN Pin: %s", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }
        close(fd);

        // 启动 I2C 读取线程
        i2c_read_thread_ = std::thread(&DartFlySystemHardwareEncoder::i2cReadThread, this);

        // 等待线程启动
        int count = 0;
        while (!thread_running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (count++ > 10)
            {
                RCLCPP_ERROR(this->get_logger(), "I2C Read Thread Start Failed!");
                i2c_read_thread_.join();
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // on_deactivate
    hardware_interface::CallbackReturn DartFlySystemHardwareEncoder::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        // 停止 I2C 读取线程
        thread_running = false;
        i2c_read_thread_.join();

        // 关闭 I2C 设备
        close(i2c_fd_);

        // 关闭 I2C Local 域隔离，向/sys/class/led/dart-i2c-en/brightness写入1
        int fd = open(dartI2CEN_path.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C Local EN Pin: %s", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (write(fd, "1", 1) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to I2C Local EN Pin: %s", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }
        close(fd);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // read
    hardware_interface::return_type DartFlySystemHardwareEncoder::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // 如果active输出角度和速度信息
        if (this->get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(),
                *get_clock(),
                1000,
                "Encoder Angle: " << encoder_angle_ << ", Encoder Velocity: " << encoder_velocity_);
        }

        return hardware_interface::return_type::OK;
    }

    // export_state_interfaces
    std::vector<hardware_interface::StateInterface> DartFlySystemHardwareEncoder::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, "position", &encoder_angle_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, "velocity", &encoder_velocity_));
        return state_interfaces;
    }
}; // namespace dart_flysystem_hardware

// Export to pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dart_flysystem_hardware::DartFlySystemHardwareEncoder, hardware_interface::SensorInterface)