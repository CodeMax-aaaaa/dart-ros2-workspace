//
// Created by cheny on 24-9-18.
//

#include "state_machine.h"
#include "openfsm.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sound_effect.h"
#include "buzzer_examples.h"
#include "motor_controller.h"
#include "micro_switch.h"
#include "dart_config.h"
#include "servo.h"

namespace state_machine {
    void FSM::update() {
        // 状态机更新
        openFSM_.update();
    }

    Dart_FSM dart_fsm;

    class ActionWaitForAllMotorOnline : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // 进入
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_dji_startup));
        }

        void update(OpenFSM &fsm) const override {
            if (motor::MotorYawLS.motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorLoad[0].motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorLoad[1].motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorTriggerLS.motor_state_ != motor::E_MotorState::DISCONNECTED &&
                motor::MotorPitchLS.motor_state_ != motor::E_MotorState::DISCONNECTED) {
                fsm.nextAction();
            }
        }

        void exit(OpenFSM &fsm) const override {
            // 退出
            soundEffectManager.clearSoundEffects();
        }
    };

    template<typename T>
    E_ActionReturnState
    actionResetLSUntilTrigger(OpenFSM &fsm,
                              motor_controller::pid_angle_velocity_controller<T> &pid,
                              E_Lead_Screw_Switch_State &triggerFlag,
                              int16_t target_reset_velocity) {
        if (pid.motor_->motor_state_ == motor::E_MotorState::DISCONNECTED) {
            fsm.enterState(E_Dart_State::Protect);
            return E_ActionReturnState::Failed;
        }
        if (pid.motor_->motor_state_ == motor::E_MotorState::RUNNING)
            if (triggerFlag == E_Lead_Screw_Switch_State::Untriggered) {
                pid.target_velocity_ = target_reset_velocity;
                pid.set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                return E_ActionReturnState::Operating;
            } else if (triggerFlag = E_Lead_Screw_Switch_State::Triggered) {
                pid.motor_->setNextState(motor::E_MotorState::IDLE);
                pid.motor_->resetRound();
                return E_ActionReturnState::Finished;
            }
    }

    // 非阻塞电机移动到底限位，返回值表示是否执行完成
    template<typename TypeTarget, typename TypeGate, typename TypeController>
    E_ActionReturnState
    actionResetMotorUntilBlocked(motor_controller::pid_angle_velocity_controller<TypeController> &controller_,
                                 TypeTarget operation_target_, TypeGate gate_velocity_,
                                 TickType_t timeout_, bool &running_flag_,
                                 bool openloop_ = false) {

        static TickType_t last_time;
        if (!running_flag_) { // 开始运行
            last_time = xTaskGetTickCount();
            if (openloop_) {
                controller_.
                        set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::OPEN_LOOP);
                controller_.
                        target_openloop_ = operation_target_;
            } else {
                controller_.
                        set_state(motor_controller::E_PID_Velocity_Angle_Controller_State::VELOCITY_CONTROL);
                controller_.
                        target_velocity_ = operation_target_;
            }
            running_flag_ = true;
            return E_ActionReturnState::Operating; // 未完成
        } else { // 运行中
            if (abs(controller_.motor_->current_velocity_) <= abs(gate_velocity_)) {
                if (xTaskGetTickCount() - last_time > timeout_) {
                    // 停止电机，复原状态
                    if (openloop_)
                        controller_.target_openloop_ = 0;
                    else
                        controller_.target_velocity_ = 0;
                    controller_.motor_->setNextState(motor::E_MotorState::IDLE);
                    running_flag_ = false;
                    return E_ActionReturnState::Finished; // 完成
                }
            } else {
                controller_.
                        target_velocity_ = operation_target_;
                last_time = xTaskGetTickCount();
            }
            return E_ActionReturnState::Operating; // 未完成
        }
    }

    class ActionResetMotors : public OpenFSMAction {
        // Dart_FSM Flags使用
        // 0 for MotorYawLS Reset Success
        // 1 for MotorPitchLS Reset Success
        // 2 for MotorTriggerLS Reset Success
    public:

        void enter(OpenFSM &fsm) const override {
            motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
            motor::MotorYawLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorPitchLS.setNextState(motor::E_MotorState::RUNNING);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::RUNNING);
            fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Success = false;
            fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Success = false;
//            trigger_servo.setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE);
        }

        void update(OpenFSM &fsm) const override {
            if (actionResetLSUntilTrigger<>(fsm, motor_controller::MotorYawLSController,
                                            yaw_switch_state,
                                            CONFIG_TARGET_RESET_VELOCITY_YAWLS) ==
                E_ActionReturnState::Finished &&
                actionResetLSUntilTrigger<>(fsm, motor_controller::MotorPitchLSController,
                                            pitch_switch_state,
                                            CONFIG_TARGET_RESET_VELOCITY_PITCHLS) ==
                E_ActionReturnState::Finished &&
                actionResetLSUntilTrigger<>(fsm, motor_controller::MotorTriggerLSController,
                                            trigger_switch_state,
                                            CONFIG_TARGET_RESET_VELOCITY_TRIGGERLS) ==
                E_ActionReturnState::Finished &&
                actionResetMotorUntilBlocked<>(motor_controller::MotorLoadController[0],
                                               CONFIG_TARGET_RESET_VELOCITY_LOAD,
                                               CONFIG_GATE_VELOCITY_LOAD,
                                               pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_LOAD),
                                               fsm.custom<Dart_FSM>()->ActionResetMotors_Load_0_Success, false) ==
                E_ActionReturnState::Finished &&
                actionResetMotorUntilBlocked<>(motor_controller::MotorLoadController[1],
                                               CONFIG_TARGET_RESET_VELOCITY_LOAD,
                                               CONFIG_GATE_VELOCITY_LOAD,
                                               pdMS_TO_TICKS(CONFIG_TIMEOUT_RESET_LOAD),
                                               fsm.custom<Dart_FSM>()->ActionResetMotors_Load_1_Success, false) ==
                E_ActionReturnState::Finished) {
                fsm.nextAction();
            }
        }
    };

    class ActionProtect : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {
            // 保护状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_protect));
        }

        void update(OpenFSM &fsm) const override {
            // 保护状态
            motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
            motor::MotorYawLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorPitchLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::IDLE);
//            trigger_servo.setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE);
        }
    };

    class ActionRemote : public OpenFSMAction {
    public:
        void enter(OpenFSM &fsm) const override {

        }

        void update(OpenFSM &fsm) const override {
            // 遥控状态
            motor::MotorLoad[0].setNextState(motor::E_MotorState::IDLE);
            motor::MotorLoad[1].setNextState(motor::E_MotorState::IDLE);
            motor::MotorYawLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorPitchLS.setNextState(motor::E_MotorState::IDLE);
            motor::MotorTriggerLS.setNextState(motor::E_MotorState::IDLE);
//            trigger_servo.setAngle(CONFIG_TRIGGER_SERVO_RELOAD_ANGLE);
        }
    };

    class ActionMatch_Enter : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    class ActionMatch_Wait : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    class ActionMatch_Launch : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    class ActionMatch_Reload : public OpenFSMAction {
        void enter(OpenFSM &fsm) const override {
            // 比赛状态
            soundEffectManager.addSoundEffect(BUZZER_NOTE(buzzer_winxp));
        }

        void update(OpenFSM &fsm) const override {
        }

        void exit(OpenFSM &fsm) const override {
        }
    };

    void Dart_FSM::start() {
        OpenFSM::RegisterAction<ActionWaitForAllMotorOnline>("ActionWaitForAllMotorOnline");
        OpenFSM::RegisterAction<ActionResetMotors>("ActionResetMotors");
        OpenFSM::RegisterAction<ActionProtect>("ActionProtect");
        OpenFSM::RegisterAction<ActionRemote>("ActionRemote");
        OpenFSM::RegisterAction<ActionMatch_Enter>("ActionMatch_Enter");
        OpenFSM::RegisterAction<ActionMatch_Wait>("ActionMatch_Wait");
        OpenFSM::RegisterAction<ActionMatch_Launch>("ActionMatch_Launch");
        OpenFSM::RegisterAction<ActionMatch_Reload>("ActionMatch_Reload");

        OpenFSM::RegisterState("StateBoot", {"ActionWaitForAllMotorOnline", "ActionResetMotors"},
                               E_Dart_State::Boot);
        OpenFSM::RegisterState("StateProtect", {"ActionProtect"}, E_Dart_State::Protect);
        OpenFSM::RegisterState("StateRemote", {"ActionRemote"}, E_Dart_State::Remote);
        OpenFSM::RegisterState("StateMatch", {"ActionMatch_Enter", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload", "ActionMatch_Wait", "ActionMatch_Launch",
                                              "ActionMatch_Reload"},
                               E_Dart_State::Match);

        OpenFSM::RegisterRelation("StateBoot", {"StateProtect"});
        OpenFSM::RegisterRelation("StateProtect", {"StateRemote", "StateMatch"});
        OpenFSM::RegisterRelation("StateRemote", {"StateProtect", "StateMatch"});
        OpenFSM::RegisterRelation("StateMatch", {"StateProtect", "StateRemote"});

        openFSM_.setCustom(this);
        openFSM_.setStates({E_Dart_State::Boot, E_Dart_State::Protect, E_Dart_State::Remote, E_Dart_State::Match});

        openFSM_.enterState(E_Dart_State::Boot);
    }

    void fsm_thread(void *parameters) {
        TickType_t last_time;

        dart_fsm.start();

        while (true) {
            dart_fsm.update();
            vTaskDelayUntil(&last_time, pdMS_TO_TICKS(1)); // 1000Hz
        }
        vTaskDelete(nullptr);
    }

} // state_machine