#include "CanDriver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <info/msg/dart_launcher_status.hpp>
#include <info/msg/dart_param.hpp>
#include <info/msg/judge.hpp>

#include "dart_config.h"
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

using namespace sockcanpp;
using namespace sockcanpp::exceptions;

struct CanSetParameter
{
    std::string param_name;
    int param_value;

    CanSetParameter()
    {
        param_name = "";
        param_value = 0;
    }

    CanSetParameter(std::string name, int value)
    {
        param_name = name;
        param_value = value;
    }
};

enum E_Dart_Can_Param_Index
{
    VELOCITY_RATIO = 1,
    VELOCITY_FW,
    VELOCITY_FW_OFFSET,
    YAW_ANGLE_WITH_ROUNDS,
    YAW_ANGLE_WITH_ROUNDS_OFFSET,
    LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_0,
    LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_1,
    LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_2,
    LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_3,
    LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_0,
    LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_1,
    LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_2,
    LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_3
};
// -0x711 参数设置 【待设置参数类型2字节+参数数据4字节+空2字节】

// NodeCanAgent class inherits from rclcpp::Node and CanDriver
class NodeCanAgent : public rclcpp::Node,
                     public CanDriver
{
private:
    std::shared_ptr<std::thread> _canReceiveThread;
    std::shared_ptr<std::thread> _canSetThread;
    std::shared_ptr<rclcpp::Publisher<info::msg::DartLauncherStatus>> _dartLauncherStatusPublisher;
    std::shared_ptr<rclcpp::Publisher<info::msg::DartParam>> _dartParamPublisher;
    std::shared_ptr<rclcpp::Subscription<info::msg::DartParam>> _dartParamCmdSubscriber;
    std::shared_ptr<rclcpp::Publisher<info::msg::Judge>> _judgePublisher;

    info::msg::DartLauncherStatus _dartLauncherStatusMsg;
    info::msg::DartParam _dartParamMsg;
    info::msg::Judge _judgeMsg;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr callback_set_parameter_handle;

    std::vector<CanSetParameter> _canSetParameters;

    std::shared_ptr<rclcpp::TimerBase> _timer;

    std::chrono::steady_clock::time_point _lastCanReceiveTime;
    std::chrono::steady_clock::time_point _lastReceiveParameterFromCanTime;
    std::chrono::steady_clock::time_point _lastReceiveParameterFromRosTime;

    std::shared_ptr<CanDriver> _canWriteDriver;

    bool
    active_can_interface()
    {
        try
        {
            RCLCPP_INFO(
                this->get_logger(), "Activating cans interface %s",
                DART_CAN_INTERFACE);
            initialiseSocketCan();
            _canWriteDriver = std::make_shared<CanDriver>(DART_CAN_INTERFACE, CAN_RAW);
        }
        // catch any exception
        catch (std::exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(), "Error activating CAN interface: %s",
                e.what());
            return false;
        }

        return true;
    }

    void decode_can_message(CanMessage canMessage)
    {
        // C板上传状态数据
        //  一 0X700 系统状态1 &发射参数 1 【电机和遥控器在线位 8位 [1字节]+
        //  模式位 1字节+堵转过程状态1字节+发射过程状态1字节+摩擦轮Target4字节】
        //  一 0X701 发射参数2【摩擦轮 TargetOffset4 字节 +Yaw角度 4字节】
        //  一 0X702 发射参数3 【Yaw 轴角度 Offset4 字节+ Yaw轴真实角度 4字节】
        //  一 0X703 发射参数4 【第一槽位 Yaw 角度Offset + 第一槽位摩擦轮Offset】
        //  一 0X704 发射参数5 【第二槽位 Yaw 角度Offset + 第二槽位摩擦轮Offset】
        //  一 0X705 发射参数6 【第三槽位 Yaw 角度Offset + 第三槽位摩擦轮Offset】
        //  一 0X706 发射参数7 【第四槽位 Yaw 角度Offset + 第四槽位摩擦轮Offset】
        //  一 0X707 发射参数8 【摩擦轮速度比】 double*10000
        //  一 0X708 裁判日志【ext_dart_client_cmd.dart_launch_opening_status +
        //  ext_game_status.game_progress +
        //  ext_dart_info.dart_remaining_time +
        //  ext_dart_client_cmd.latest_launch_cmd_time +
        //  ext_game_status.stage_remain_time + 在线判断位】
        // 转发到话题/dart_status/system_status
        if ((int)canMessage.getCanId() >= 0x700 && (int)canMessage.getCanId() <= 0x708)
        {
            _lastCanReceiveTime = std::chrono::steady_clock::now();
            switch (canMessage.getRawFrame().can_id)
            {
            case 0x700:
            {
                // 第一位从高位到低位分别为：rc_online FW[3] FW[2] FW[1] FW[0] DM Y LS
                can_frame frame = canMessage.getRawFrame();
                uint8_t data = frame.data[0];
                _dartLauncherStatusMsg.rc_online = (data & 0x80);
                _dartLauncherStatusMsg.motor_fw_online[3] = (data & 0x40);
                _dartLauncherStatusMsg.motor_fw_online[2] = (data & 0x20);
                _dartLauncherStatusMsg.motor_fw_online[1] = (data & 0x10);
                _dartLauncherStatusMsg.motor_fw_online[0] = (data & 0x08);
                _dartLauncherStatusMsg.motor_dm_online = (data & 0x04);
                _dartLauncherStatusMsg.motor_y_online = (data & 0x02);
                _dartLauncherStatusMsg.motor_ls_online = (data & 0x01);

                _dartLauncherStatusMsg.dart_state = frame.data[1];

                _dartLauncherStatusMsg.motor_y_resetting = frame.data[2] & 0x01;
                _dartLauncherStatusMsg.motor_dm_resetting = frame.data[2] & 0x02;
                _dartLauncherStatusMsg.motor_ls_resetting = frame.data[2] & 0x04;

                _dartLauncherStatusMsg.dart_launch_process = frame.data[3];

                _dartParamMsg.target_fw_velocity = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                break;
            }
            case 0x701:
            {
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_fw_velocity_offset = (frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                _dartParamMsg.target_yaw_angle = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                break;
            }
            case 0x702:
            {
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_yaw_angle_offset = (frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                _dartLauncherStatusMsg.motor_y_angle = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                break;
            }
            case 0x703:
            {
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_yaw_launch_angle_offset[0] = (frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                _dartParamMsg.target_fw_velocity_launch_offset[0] = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                break;
            }
            case 0x704:
            {
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_yaw_launch_angle_offset[1] = (frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                if (_dartParamMsg.target_yaw_launch_angle_offset[1] != this->get_parameter("target_yaw_launch_angle_offset").as_integer_array()[1])
                    _canSetParameters.push_back(CanSetParameter{"target_yaw_launch_angle_offset", (_dartParamMsg.target_yaw_launch_angle_offset[1])});
                _dartParamMsg.target_fw_velocity_launch_offset[1] = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                if (_dartParamMsg.target_yaw_launch_angle_offset[1] != this->get_parameter("target_yaw_launch_angle_offset").as_integer_array()[1])
                    _canSetParameters.push_back(CanSetParameter{"target_yaw_launch_angle_offset", (_dartParamMsg.target_yaw_launch_angle_offset[1])});
                break;
            }
            case 0x705:
            {
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_yaw_launch_angle_offset[2] = (frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                _dartParamMsg.target_fw_velocity_launch_offset[2] = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                break;
            }
            case 0x706:
            {
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_yaw_launch_angle_offset[3] = (frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3];
                _dartParamMsg.target_fw_velocity_launch_offset[3] = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
                break;
            }
            case 0x707:
            {
                // 摩擦轮速度比
                can_frame frame = canMessage.getRawFrame();
                _dartParamMsg.target_fw_velocity_ratio = ((frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3]) / 10000.0;
                break;
            }
            case 0x708:
            {
                can_frame frame = canMessage.getRawFrame();
                _judgeMsg.dart_launch_opening_status = frame.data[0];
                _judgeMsg.game_progress = frame.data[1];
                _judgeMsg.dart_remaining_time = frame.data[2];
                _judgeMsg.latest_launch_cmd_time = (frame.data[3] << 8) + frame.data[4];
                _judgeMsg.stage_remain_time = (frame.data[5] << 8) + frame.data[6];
                _judgeMsg.judge_online = frame.data[7];
                _dartLauncherStatusMsg.judge_online = frame.data[7];
                break;
            }
            default:
                break;
            }
        }
        // 摩擦轮速度反馈
        else if ((int)canMessage.getCanId() >= 0x901 && (int)canMessage.getCanId() <= 0x904)
        {
            can_frame frame = canMessage.getRawFrame();
            _dartLauncherStatusMsg.motor_fw_velocity[(int)canMessage.getCanId() - 0x901] = ((frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3]) / 7;
        }
        // 母线电压
        else if ((int)canMessage.getCanId() >= 0x1B01 && (int)canMessage.getCanId() <= 0x1B04)
        {
            can_frame frame = canMessage.getRawFrame();
            _dartLauncherStatusMsg.bus_voltage[(int)canMessage.getCanId() - 0x1B01] = ((frame.data[4] << 8) | frame.data[5]) / 10.0;
        }
    }

    std::string stringToHex(const uint8_t &input)
    {
        std::stringstream ss;
        // 使用hex来设置stringstream以16进制输出
        ss << std::hex << std::setfill('0');

        ss << std::setw(2) << static_cast<int>(input);

        return ss.str();
    }

    void setParameter(int can_id, int param_index, int param_value)
    {
        can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        frame.data[0] = (int16_t(param_index) >> 8) & 0xFF;
        frame.data[1] = int16_t(param_index) & 0xFF;
        // 大端模式
        frame.data[2] = ((int32_t)param_value >> 24) & 0x000000FF;
        frame.data[3] = ((int32_t)param_value >> 16) & 0x000000FF;
        frame.data[4] = ((int32_t)param_value >> 8) & 0x000000FF;
        frame.data[5] = (int32_t)param_value & 0x000000FF;
        frame.data[6] = 0;
        frame.data[7] = 0;
        _canWriteDriver->sendMessage(CanMessage(frame));
    }

    void checkParamChanged()
    {
        // check every parameter in _dartParamMsg and check if it is different from the ros parameter
        if (_dartParamMsg.target_yaw_angle != this->get_parameter("target_yaw_angle").as_int())
            _canSetParameters.push_back(CanSetParameter{"target_yaw_angle", this->get_parameter("target_yaw_angle").as_int()});
        if (_dartParamMsg.target_yaw_angle_offset != this->get_parameter("target_yaw_angle_offset").as_int())
            _canSetParameters.push_back(CanSetParameter{"target_yaw_angle_offset", this->get_parameter("target_yaw_angle_offset").as_int()});
        if (_dartParamMsg.target_fw_velocity != this->get_parameter("target_fw_velocity").as_int())
            _canSetParameters.push_back(CanSetParameter{"target_fw_velocity", this->get_parameter("target_fw_velocity").as_int()});
        if (_dartParamMsg.target_fw_velocity_offset != this->get_parameter("target_fw_velocity_offset").as_int())
            _canSetParameters.push_back(CanSetParameter{"target_fw_velocity_offset", this->get_parameter("target_fw_velocity_offset").as_int()});
        if (int(_dartParamMsg.target_fw_velocity_ratio * 10000) != int(this->get_parameter("target_fw_velocity_ratio").as_double() * 10000))
            _canSetParameters.push_back(CanSetParameter{"target_fw_velocity_ratio", this->get_parameter("target_fw_velocity_ratio").as_double() * 10000});
        for (size_t i = 0; i < 4; i++)
        {
            if (_dartParamMsg.target_yaw_launch_angle_offset[i] != this->get_parameter("target_yaw_launch_angle_offset").as_integer_array()[i])
                _canSetParameters.push_back(CanSetParameter{"target_yaw_launch_angle_offset" + std::to_string(i), this->get_parameter("target_yaw_launch_angle_offset").as_integer_array()[i]});
            if (_dartParamMsg.target_fw_velocity_launch_offset[i] != this->get_parameter("target_fw_velocity_launch_offset").as_integer_array()[i])
                _canSetParameters.push_back(CanSetParameter{"target_fw_velocity_launch_offset" + std::to_string(i), this->get_parameter("target_fw_velocity_launch_offset").as_integer_array()[i]});
        }
    }

    void callback_set_parameter(const std::vector<rclcpp::Parameter> &parameters)
    {
        _lastReceiveParameterFromRosTime = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Received params from ROS");
    }

    void loadParametersfromMsg(const info::msg::DartParam::SharedPtr msg)
    {
        // 取消回调函数
        callback_set_parameter_handle.reset();
        DartConfig::loadParametersfromMsg(*this, msg);
        // 重新注册回调函数
        callback_set_parameter_handle = this->add_post_set_parameters_callback(std::bind(&NodeCanAgent::callback_set_parameter, this, std::placeholders::_1));
    }

    void canSetThread()
    {
        static bool _first_boot = true;
        while (rclcpp::ok())
        {
            checkParamChanged();
            if (_canSetParameters.size() == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            else if (_dartLauncherStatusMsg.dart_state == 100) // boot
            {
                _first_boot = true;
                while(_dartLauncherStatusMsg.dart_state == 100)
                {
                    RCLCPP_INFO(this->get_logger(), "Waiting for launcher to boot");
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }
            else if (_lastReceiveParameterFromCanTime == _lastReceiveParameterFromRosTime) // CAN数据率先更新
                _lastReceiveParameterFromCanTime = std::chrono::steady_clock::now();

            for (auto &param : _canSetParameters)
            {
                RCLCPP_INFO(this->get_logger(), "Different parameter: %s , value %d", param.param_name.c_str(), param.param_value);
            }

            // Can parameters are updated from the can bus
            if (_lastReceiveParameterFromCanTime > _lastReceiveParameterFromRosTime && _first_boot == false)
            {
                RCLCPP_INFO(this->get_logger(), "Setting parameter from CAN");
                // update _dartParamMsg from can bus to ros parameter
                loadParametersfromMsg(std::make_shared<info::msg::DartParam>(_dartParamMsg));
                // send _dartParamMsg to topic
                static int64_t frame_id = 0;
                _dartParamMsg.header.stamp = this->now();
                _dartParamMsg.header.frame_id = std::to_string(frame_id++);
                _dartParamPublisher->publish(_dartParamMsg);
                _lastReceiveParameterFromRosTime = std::chrono::steady_clock::now();
                _lastReceiveParameterFromCanTime = _lastReceiveParameterFromRosTime;
            }
            else if (_lastReceiveParameterFromCanTime < _lastReceiveParameterFromRosTime || _first_boot == true)
            {
                RCLCPP_INFO(this->get_logger(), "Setting parameter from ROS");

                for (auto &param : _canSetParameters)
                {
                    RCLCPP_DEBUG(this->get_logger(), "Setting parameter %s to %d", param.param_name.c_str(), param.param_value);
                    // set parameter to can bus
                    if (param.param_name == "target_yaw_angle")
                    {
                        setParameter(0x711, YAW_ANGLE_WITH_ROUNDS, param.param_value);
                    }
                    else if (param.param_name == "target_yaw_angle_offset")
                    {
                        setParameter(0x711, YAW_ANGLE_WITH_ROUNDS_OFFSET, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity")
                    {
                        setParameter(0x711, VELOCITY_FW, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity_offset")
                    {
                        setParameter(0x711, VELOCITY_FW_OFFSET, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity_ratio")
                    {
                        setParameter(0x711, VELOCITY_RATIO, param.param_value);
                    }
                    else if (param.param_name == "target_yaw_launch_angle_offset0")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_0, param.param_value);
                    }
                    else if (param.param_name == "target_yaw_launch_angle_offset1")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_1, param.param_value);
                    }
                    else if (param.param_name == "target_yaw_launch_angle_offset2")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_2, param.param_value);
                    }
                    else if (param.param_name == "target_yaw_launch_angle_offset3")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_YAW_ANGLE_WITH_ROUNDS_OFFSET_3, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity_launch_offset0")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_0, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity_launch_offset1")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_1, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity_launch_offset2")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_2, param.param_value);
                    }
                    else if (param.param_name == "target_fw_velocity_launch_offset3")
                    {
                        setParameter(0x711, LAUNCH_PARAMS_TARGET_VELOCITY_FW_OFFSET_3, param.param_value);
                    }
                }
                auto timepoint = std::chrono::steady_clock::now();
                while (rclcpp::ok())
                {
                    // 检查是否有参数未设置成功
                    _canSetParameters.clear();
                    checkParamChanged();
                    if (_canSetParameters.size() == 0)
                    {
                        if (_first_boot)
                        {
                            _first_boot = false;
                            RCLCPP_INFO(this->get_logger(), "First boot param set successfully");
                        }
                        else
                            RCLCPP_INFO(this->get_logger(), "All parameters set successfully");
                        _lastReceiveParameterFromRosTime = std::chrono::steady_clock::now();
                        _lastReceiveParameterFromCanTime = _lastReceiveParameterFromRosTime;

                        break;
                    }
                    else
                    {
                        for (auto &param : _canSetParameters)
                        {
                            RCLCPP_DEBUG(this->get_logger(), "Failed to set parameter %s to %d, retrying", param.param_name.c_str(), param.param_value);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    // 超时检查
                    if (std::chrono::steady_clock::now() - timepoint > std::chrono::seconds(5))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to set parameters, timeout");
                        _lastReceiveParameterFromCanTime = std::chrono::steady_clock::now(); // 为了让参数恢复到can给出的值
                        break;
                    }
                }
                _canSetParameters.clear();
            }
            _canSetParameters.clear();
        }
    }

    void
    canReceiveThread()
    {
        while (rclcpp::ok())
        {
            try
            {
                CanMessage canMessage = readMessage();
                std::stringstream ss;
                for (auto byte : canMessage.getFrameData())
                {
                    ss << std::hex << std::setw(2) << std::setfill('0')
                       << (int)byte << " ";
                }
                RCLCPP_DEBUG_STREAM(
                    this->get_logger(),
                    "Received CAN message: [" << std::hex << canMessage.getRawFrame().can_id
                                              << std::hex << "] Data=" << ss.str());

                // decode can message
                decode_can_message(canMessage);
            }
            catch (std::exception &e)
            {
                RCLCPP_ERROR(
                    this->get_logger(), "Error receiving CAN message: %s",
                    e.what());
            }
        }
    }

public:
    NodeCanAgent() : Node("can_agent"), CanDriver()
    {
        // 设置参数回调函数
        callback_set_parameter_handle = this->add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters)
                                                                               { _lastReceiveParameterFromRosTime = std::chrono::steady_clock::now(); });

        DartConfig::declareParameters(*this);
        _lastReceiveParameterFromCanTime = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "NodeCanAgent up");
        _defaultSenderId = 0;
        _canFilterMask = 0;
        _canProtocol = CAN_RAW;
        _canInterface = DART_CAN_INTERFACE;
        while (!active_can_interface())
        {
            if (!rclcpp::ok())
                rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface activated");

        // 设置参数回调函数
        callback_set_parameter_handle.reset();
        callback_set_parameter_handle = this->add_post_set_parameters_callback(std::bind(&NodeCanAgent::callback_set_parameter, this, std::placeholders::_1));

        // 初始化节点消息
        _dartLauncherStatusPublisher = this->create_publisher<info::msg::DartLauncherStatus>(
            "/dart_controller/dart_launcher_status", 10);

        _dartParamPublisher = this->create_publisher<info::msg::DartParam>(
            "/dart_controller/dart_launcher_present_param",
            rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());

        _dartParamCmdSubscriber = this->create_subscription<info::msg::DartParam>(
            "/dart_controller/dart_launcher_param_cmd",
            rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
            [this](const info::msg::DartParam::SharedPtr msg)
            {
                // 设置参数
                loadParametersfromMsg(msg);
                RCLCPP_INFO(this->get_logger(), "Received params MSG from ROS");
                _lastReceiveParameterFromRosTime = std::chrono::steady_clock::now();
            });

        _judgePublisher = this->create_publisher<info::msg::Judge>(
            "/dart_controller/judge", 10);

        _judgeMsg = info::msg::Judge();

        _dartLauncherStatusMsg = info::msg::DartLauncherStatus();
        _dartParamMsg = info::msg::DartParam();

        // 开启CAN总线收发线程
        _canReceiveThread = std::make_shared<std::thread>(&NodeCanAgent::canReceiveThread, this);
        _canSetThread = std::make_shared<std::thread>(&NodeCanAgent::canSetThread, this);

        // 开启定时器
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]()
            {
                // 判断是否断联
                if (std::chrono::steady_clock::now() - _lastCanReceiveTime > std::chrono::milliseconds(500))
                {
                    _dartLauncherStatusMsg.dart_launcher_online = false;
                    _dartLauncherStatusMsg.judge_online = false;
                    _judgeMsg.judge_online = false;
                }
                else
                    _dartLauncherStatusMsg.dart_launcher_online = true;

                static int64_t frame_id = 0;

                // 更新Header
                _dartLauncherStatusMsg.header.stamp = this->now();
                _dartLauncherStatusMsg.header.frame_id = std::to_string(frame_id++);
                _judgeMsg.header.stamp = this->now();
                _judgeMsg.header.frame_id = std::to_string(frame_id++);

                // 发布消息
                _dartLauncherStatusPublisher->publish(_dartLauncherStatusMsg);
                _judgePublisher->publish(_judgeMsg);
            });
    }

    void canDisconnectCallback()
    {
        RCLCPP_FATAL(this->get_logger(), "CAN interface disconnected, reactivating...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uninitialiseSocketCan();
        _canWriteDriver.reset();
        while (rclcpp::ok())
        {
            if (active_can_interface())
            {
                RCLCPP_INFO(this->get_logger(), "CAN interface reactivated");
                break;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "CAN interface reactivation failed");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    ~NodeCanAgent()
    {
        RCLCPP_INFO(this->get_logger(), "NodeCanAgent down");
    }
};

std::shared_ptr<NodeCanAgent> node;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<NodeCanAgent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}