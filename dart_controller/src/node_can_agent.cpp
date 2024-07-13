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

// NodeCanAgent class inherits from rclcpp::Node and CanDriver
class NodeCanAgent : public rclcpp::Node, public CanDriver
{
private:
    std::shared_ptr<std::thread> _canReceiveThread;
    std::shared_ptr<rclcpp::Publisher<info::msg::DartLauncherStatus>> _dartLauncherStatusPublisher;
    std::shared_ptr<rclcpp::Publisher<info::msg::DartParam>> _dartParamPublisher;
    std::shared_ptr<rclcpp::Publisher<info::msg::Judge>> _judgePublisher;
    info::msg::DartLauncherStatus _dartLauncherStatusMsg;
    info::msg::DartParam _dartParamMsg;
    info::msg::Judge _judgeMsg;

    std::shared_ptr<rclcpp::TimerBase> _timer;

    std::chrono::steady_clock::time_point _lastCanReceiveTime;

    bool
    active_can_interface()
    {
        try
        {
            RCLCPP_INFO(
                this->get_logger(), "Activating can interface %s",
                DART_CAN_INTERFACE);

            initialiseSocketCan();
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
        if ((int)canMessage.getCanId() >= 0x700 && (int)canMessage.getCanId() <= 0x709)
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
                _dartParamMsg.target_fw_velocity_launch_offset[1] = (frame.data[4] << 24) + (frame.data[5] << 16) + (frame.data[6] << 8) + frame.data[7];
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
                _dartParamMsg.target_fw_velocity_ratio = ((frame.data[0] << 24) + (frame.data[1] << 16) + (frame.data[2] << 8) + frame.data[3]) / 10000.0;
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
            _dartLauncherStatusMsg.motor_fw_velocity[frame.can_id - 0x901] = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
        }
        // 母线电压
        else if ((int)canMessage.getCanId() >= 0x1B00 && (int)canMessage.getCanId() <= 0x1BFF)
        {
            can_frame frame = canMessage.getRawFrame();
            _dartLauncherStatusMsg.bus_voltage[frame.can_id - 0x1B00] = ((frame.data[4] << 8) | frame.data[5]) / 10.0;
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
        RCLCPP_INFO(this->get_logger(), "NodeCanAgent up");
        _defaultSenderId = 0;
        _canFilterMask = 0;
        _canProtocol = CAN_RAW;
        _canInterface = DART_CAN_INTERFACE;
        while (!active_can_interface())
        {
        }
        RCLCPP_INFO(this->get_logger(), "CAN interface activated");

        // 初始化节点消息
        _dartLauncherStatusPublisher = this->create_publisher<info::msg::DartLauncherStatus>(
            "/dart_controller/dart_launcher_status", 10);

        _dartParamPublisher = this->create_publisher<info::msg::DartParam>(
            "/dart_controller/dart_launcher_present_param", 10);

        _judgePublisher = this->create_publisher<info::msg::Judge>(
            "/dart_controller/judge", 10);

        _judgeMsg = info::msg::Judge();

        _dartLauncherStatusMsg = info::msg::DartLauncherStatus();
        _dartParamMsg = info::msg::DartParam();

        // 开启CAN总线接收线程
        _canReceiveThread = std::make_shared<std::thread>(&NodeCanAgent::canReceiveThread, this);

        // 开启定时器
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]()
            {
                // 判断是否断联
                if (std::chrono::steady_clock::now() - _lastCanReceiveTime > std::chrono::milliseconds(500))
                    _dartLauncherStatusMsg.dart_launcher_online = false;
                else
                    _dartLauncherStatusMsg.dart_launcher_online = true;

                static int64_t frame_id = 0;

                // 更新Header
                _dartLauncherStatusMsg.header.stamp = this->now();
                _dartLauncherStatusMsg.header.frame_id = std::to_string(frame_id++);
                _dartParamMsg.header.stamp = this->now();
                _dartParamMsg.header.frame_id = std::to_string(frame_id++);
                _judgeMsg.header.stamp = this->now();
                _judgeMsg.header.frame_id = std::to_string(frame_id++);

                // 发布消息
                _dartLauncherStatusPublisher->publish(_dartLauncherStatusMsg);
                _dartParamPublisher->publish(_dartParamMsg);
                _judgePublisher->publish(_judgeMsg);
            });
    }

    void canDisconnectCallback()
    {
        RCLCPP_FATAL(this->get_logger(), "CAN interface disconnected, reactivating...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uninitialiseSocketCan();
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