/**
 * @file linux_pwm.cpp
 * @brief Linux PWM硬件控制库，使用sysfs接口
 */

#include "dart_flysystem_hardware/linux_pwm.hpp"

// sysfs接口
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <sys/stat.h>

#include <cstring>
#include "rclcpp/rclcpp.hpp"

namespace linuxPWM
{

    bool LinuxPwm::begin(int chip, int channel) {
        pwm_chip_ = chip;
        pwm_channel_ = channel;
        // 生成各种PATH
        pwm_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_);
        RCLCPP_INFO(rclcpp::get_logger("LinuxPwm"), "Lib construction, PWM Path: %s", pwm_path_.c_str());
    
        std::string pwm_export_path_ = pwm_path_ + "/export";
        // 决定是否导出PWM，若已导出则不再导出
        std::string pwm_channel_path_ = pwm_path_ + "/pwm" + std::to_string(pwm_channel_);
        if (access(pwm_channel_path_.c_str(), F_OK) == -1) {
            int fd = open(pwm_export_path_.c_str(), O_WRONLY);
            if (fd == -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open export for PWM: %s", strerror(errno));
                return false;
            }
            std::string channel_str = std::to_string(pwm_channel_);
            if (write(fd, channel_str.c_str(), channel_str.size()) == -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to export for PWM: %s", strerror(errno));
            }
            close(fd);
            // 设置权限
            std::string pwm_channel_permission_path_[4] = {
                pwm_channel_path_ + "/enable",
                pwm_channel_path_ + "/duty_cycle",
                pwm_channel_path_ + "/period",
                pwm_channel_path_ + "/polarity"
            };
            for (int i = 0; i < 4; i++) {
                if (system(("sudo chmod 666 " + pwm_channel_permission_path_[i]).c_str()) == -1) {
                    RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to set permission for PWM: %s", strerror(errno));
                    return false;
                }
            }
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("LinuxPwm"), "PWM channel %d already exported", pwm_channel_);

        return true;
    }

    LinuxPwm::~LinuxPwm()
    {
        std::string unexport_path_ = pwm_path_ + "/unexport";
        int fd = open(unexport_path_.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open unexport for PWM: %s", strerror(errno));
            return;
        }
        std::string channel_str_ = std::to_string(pwm_channel_);
        if (write(fd, channel_str_.c_str(), channel_str_.size()) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to unexport for PWM: %s", strerror(errno));
        }
        close(fd);
    }

    bool LinuxPwm::enable() const {
        // Write 1 to ${pwm_path_}/pwm${pwm_channel_}/enable
        std::string enable_path_ = pwm_path_ + "/pwm" + std::to_string(pwm_channel_) + "/enable";
        int fd = open(enable_path_.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open enable for PWM: %s", strerror(errno));
            return false;
        }
        if (write(fd, "1", 1) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to enable for PWM: %s", strerror(errno));
            return false;
        }
        // Write normal polarity
        std::string polarity_path_ = pwm_path_ + "/pwm" + std::to_string(pwm_channel_) + "/polarity";
        fd = open(polarity_path_.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open polarity for PWM: %s", strerror(errno));
            return false;
        }
        if (write(fd, "normal", 6) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to polarity for PWM: %s", strerror(errno));
            return false;
        }
        close(fd);
        return true;
    }

    bool LinuxPwm::disable() const {
        std::string disable_path_ = pwm_path_ + "/pwm" + std::to_string(pwm_channel_) + "/enable";
        int fd = open(disable_path_.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open disable for PWM: %s", strerror(errno));
            return false;
        }
        if (write(fd, "0", 1) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to disable for PWM: %s", strerror(errno));
            return false;
        }
        close(fd);
        return true;
    }

    bool LinuxPwm::setDutyCycle(unsigned int duty_cycle_ns) const
    {
        const std::string duty_cycle_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_) + "/pwm" + std::to_string(pwm_channel_) + "/duty_cycle";
        int fd = open(duty_cycle_path_.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open duty_cycle for PWM: %s", strerror(errno));
            return false;
        }
        std::string duty_cycle_str_ = std::to_string(duty_cycle_ns);
        if (write(fd, duty_cycle_str_.c_str(), duty_cycle_str_.size()) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to duty_cycle for PWM: %s", strerror(errno));
            return false;
        }
        close(fd);
        return true;
    }

    bool LinuxPwm::setPeriod(unsigned int period_ns) const
    {
        const std::string period_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_) + "/pwm" + std::to_string(pwm_channel_) + "/period";
        int fd = open(period_path_.c_str(), O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to open period for PWM: %s", strerror(errno));
            return false;
        }
        const std::string period_str_ = std::to_string(period_ns);
        if (write(fd, period_str_.c_str(), period_str_.size()) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LinuxPwm"), "Failed to write to period for PWM: %s", strerror(errno));
            return false;
        }
        close(fd);
        return true;
    }

} // namespace linuxPWM