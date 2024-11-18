/**
 * @file linux_pwm.hpp
 * @brief Linux PWM硬件控制库，使用sysfs接口
 */
#ifndef LINUX_PWM_HPP
#define LINUX_PWM_HPP

#include <string>

namespace linuxPWM
{

    class LinuxPwm
    {
    public:
        LinuxPwm() = default;
        ~LinuxPwm();

        bool begin(int pwm_chip, int pwm_channel);
        bool enable() const;
        bool disable() const;
        bool setPeriod(unsigned int period_ns) const;
        bool setDutyCycle(unsigned int duty_cycle_ns) const;

    private:
        int pwm_chip_;
        int pwm_channel_;
        int initial_angle_;
        std::string pwm_path_;
    };
}
#endif // LINUX_PWM_HPP