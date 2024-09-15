//
// Created by cheny on 24-9-11.
//

#include "motor_controller.h"

namespace motor_controller {
    template<typename T>
    pid_controller<T>::pid_controller(T kp, T ki, T kd, T sum_error_max, T p_max, T i_max, T output_max)
            :
            kp(kp), ki(ki), kd(kd), sum_error_max(sum_error_max), p_max(p_max), i_max(i_max),
            output_max(output_max) {
        cur_error = 0;
        last_error = 0;
        sum_error = 0;
        output = 0;
    }

    template<typename T>
    T pid_controller<T>::update(T current) {
        last_error = cur_error;
        cur_error = target - current;
        sum_error = LIMIT_MIN_MAX(sum_error + cur_error,
                                  -sum_error_max,
                                  sum_error_max);

        output = LIMIT_MIN_MAX((kp * cur_error), -p_max, p_max) +
                 LIMIT_MIN_MAX((ki * sum_error), -i_max, i_max) +
                 kd * (cur_error - last_error);

        output = LIMIT_MIN_MAX(output, -output_max, output_max);

        return output;
    }

    template<typename T>
    void pid_controller<T>::reset() {
        cur_error = 0;
        last_error = 0;
        sum_error = 0;
        output = 0;
    }

    template<typename T>
    pid_angle_velocity_controller<T>::pid_angle_velocity_controller(pid_controller<T> pid_velocity,
                                                                    pid_controller<T> pid_angle,
                                                                    motor::motor_rm *motor,
                                                                    E_PID_Velocity_Angle_Controller_State state
    )  : pid_velocity_(pid_velocity), pid_angle_(pid_angle), motor_(motor), state_(state), target_velocity_(0){
        target_angle_with_rounds_ = 0;
        current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
    }

    template<typename T>
    T pid_angle_velocity_controller<T>::update() {
        if (state_ == VELOCITY_CONTROL) {
            current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
            current_velocity_ = motor_->current_velocity_ * 0.6 + current_velocity_ * 0.4;
            pid_velocity_.target = target_velocity_;
            return pid_velocity_.update(current_velocity_);
        } else if (state_ == ANGLE_CONTROL) {
            // 限幅
            LIMIT_MIN_MAX(target_angle_with_rounds_, 0, 370000);
            current_angle_with_rounds_ = motor_->current_round_ * 8192 + motor_->current_angle_;
            current_velocity_ = motor_->current_velocity_ * 0.6 + current_velocity_ * 0.4;
            pid_angle_.target = target_angle_with_rounds_;
            pid_velocity_.target = -pid_angle_.update(current_angle_with_rounds_);
            return pid_velocity_.update(current_velocity_);
        } else {
            return target_openloop_;
        }
    }

    template<typename T>
    void pid_angle_velocity_controller<T>::set_state(E_PID_Velocity_Angle_Controller_State state) {
        if (state_ != state) {
            state_ = state;
            reset();
        }
    }

    template<typename T>
    void pid_angle_velocity_controller<T>::reset() {
        pid_velocity_.reset();
        pid_angle_.reset();
    }

    [[noreturn]] void pid_control_task(void *pvParameter) {
// 1KHz
        // Initialize PID Controller
        TickType_t xLastWakeTime = xTaskGetTickCount();
        static uint32_t tx_mailbox;
        // Can Frame
        CAN_TxHeaderTypeDef tx_header;
        tx_header.StdId = 0x200;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 8;
        tx_header.TransmitGlobalTime = DISABLE;

        uint8_t can_array[8];
        while (true) {
            // Update Controller
            if (motor::MotorYawLS.motor_state_ == motor::RUNNING) {
                motor::MotorYawLS.setCurrent(MotorYawLSController.update());
            } else {
                MotorYawLSController.reset();
            }
            motor::MotorYawLS.updateCurrent(); // Will detect Motor Next State Automatically

            vTaskDelayUntil(&xLastWakeTime, 1);
        }
        vTaskDelete(nullptr);
    }
}