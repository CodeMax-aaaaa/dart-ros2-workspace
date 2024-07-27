#include <node_lvgl_ui.hpp>

#include <custom.h>

#include <chrono>

using namespace std::chrono_literals;
using namespace DartConfig;

lv_ui guider_ui;
std::shared_ptr<NodeLVGLUI> node;

NodeLVGLUI::NodeLVGLUI() : Node("lvgl_ui")
{
  // 参数
  this->declare_parameter("yaw_angle_calibration_factor", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->set_parameter(rclcpp::Parameter("yaw_angle_calibration_factor", 20.0));
  this->declare_parameter("yaw_angle_calibration_filter_factor", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->set_parameter(rclcpp::Parameter("yaw_angle_calibration_filter_factor", 0.95));

  lv_log_register_print_cb(print_cb);

  lv_init();

  /*Linux frame buffer device init*/
  fbdev_init();

  /*A small buffer for LittlevGL to draw the screen's content*/
  static lv_color_t buf[2][DISP_BUF_SIZE];

  /*Initialize a descriptor for the buffer*/
  static lv_disp_draw_buf_t disp_buf;
  lv_disp_draw_buf_init(&disp_buf, buf[0], buf[1], DISP_BUF_SIZE);

  /*Initialize and register a display driver*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &disp_buf;
  disp_drv.flush_cb = fbdev_flush;
  disp_drv.hor_res = 1024;
  disp_drv.ver_res = 600;
  lv_disp_drv_register(&disp_drv);

  evdev_init();

  static lv_indev_drv_t indev_drv_1;
  lv_indev_drv_init(&indev_drv_1); /*Basic initialization*/
  indev_drv_1.type = LV_INDEV_TYPE_POINTER;

  /*This function will be called periodically (by the library) to get the mouse position and state*/
  indev_drv_1.read_cb = evdev_read;
  lv_indev_drv_register(&indev_drv_1);

  /*
    GUI Guider
   */
  setup_ui(&guider_ui);
  events_init(&guider_ui);
  custom_init(&guider_ui);

  DartConfig::declareParameters(*this);
  // Subscribe /dart_controller/dart_launcher_status /dart_controller/dart_launcher_present_param
  // Publish /dart_controller/dart_launcher_cmd
  dart_launcher_status_sub_ = this->create_subscription<info::msg::DartLauncherStatus>(
      "/dart_controller/dart_launcher_status",
      10, std::bind(&NodeLVGLUI::update_dart_launcher_status_callback, this, std::placeholders::_1));

  dart_launcher_present_param_sub_ = this->create_subscription<info::msg::DartParam>(
      "/dart_controller/dart_launcher_present_param",
      rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(), std::bind(&NodeLVGLUI::update_dart_launcher_present_param_callback, this, std::placeholders::_1));

  dart_launcher_cmd_pub_ = this->create_publisher<info::msg::DartParam>(
      "/dart_controller/dart_launcher_param_cmd",
      rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());

  green_light_sub_ = this->create_subscription<info::msg::GreenLight>(
      "/detect/locate",
      rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
      std::bind(&NodeLVGLUI::update_green_light_callback, this, std::placeholders::_1));

  cv_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "detect/image", 1, std::bind(&NodeLVGLUI::update_cv_image, this, std::placeholders::_1));

  static auto callback_handle_ = this->add_post_set_parameters_callback(std::bind(&NodeLVGLUI::set_parameters_callback, this, std::placeholders::_1));

  update_ip_address();
  update_parameters_to_gui();

  RCLCPP_INFO(this->get_logger(), "UI thread started");
  /*Handle LVGL tasks*/
  auto timer_callback_ui =
      [this]() -> void
  {
    while (rclcpp::ok())
    {
      this->mutex_ui_.lock(); // Lock the mutex to prevent the GUI from being updated while the screen is being cleaned
      lv_timer_handler();
      this->mutex_ui_.unlock();
      std::this_thread::sleep_for(5ms);
    }
    this->mutex_ui_.lock();
    lv_obj_clean(lv_scr_act()); // Clean up the active screen
    lv_timer_handler();         // Call the timer handler
    lv_deinit();                // Deinitialize LittlevGL
    this->mutex_ui_.unlock();
  };

  auto timer_callback_ip_update =
      [this]() -> void
  {
    update_ip_address();
  };

  auto timer_callback_lv_tick_inc =
      [this]() -> void
  {
    lv_tick_inc(1);
  };

  timer_[0] = this->create_wall_timer(1000ms, timer_callback_ip_update);
  // timer_[1] = this->create_wall_timer(1s, timer_callback_informational_update);
  timer_[1] = this->create_wall_timer(1ms, timer_callback_lv_tick_inc);

  static std::thread ui_thread(timer_callback_ui);
  ui_thread.detach();
}

void NodeLVGLUI::loadParametersfromGUI()
{
  // generate a msg from GUI
  info::msg::DartParam msg;
  msg.target_yaw_angle = lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_angle);
  msg.target_yaw_angle_offset = lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_offset);
  msg.target_fw_velocity = lv_spinbox_get_value(guider_ui.Main_spinbox_fw_speed);
  msg.target_fw_velocity_offset = lv_spinbox_get_value(guider_ui.Main_spinbox_fw_speed_offset);
  msg.target_fw_velocity_ratio = lv_spinbox_get_value(guider_ui.Main_spinbox_fw_speed_ratio) / 1000.0;
  msg.auto_yaw_calibration = lv_obj_has_state(guider_ui.Main_sw_auto_yaw_calibration, LV_STATE_CHECKED);
  msg.auto_fw_calibration = lv_obj_has_state(guider_ui.Main_sw_auto_rpm_calibration, LV_STATE_CHECKED);
  msg.target_yaw_x_axis = lv_spinbox_get_value(guider_ui.Main_spinbox_yaw_calibration_x) / 100.0;
  dart_launcher_cmd_pub_->publish(msg);

  callback_disabled = true;
  loadParametersfromMsg(*this, std::make_shared<info::msg::DartParam>(msg));
  callback_disabled = false;
}

void NodeLVGLUI::calibration_yaw()
{
  // 检测参数内的Yaw轴x方向目标角度，如果有效检测到绿灯，则计算绿灯坐标与目标角度的差值，作为偏移量，进行折算后加到目标角度上
  // 如果未检测到绿灯，则不动
  if (dart_launcher_status_->dart_launcher_online && dart_launcher_status_->dart_state != 100 && dart_launcher_status_->dart_state != 101) // 未处于boot/保护状态
  {
    int32_t target_yaw_angle = this->get_parameter("target_yaw_angle").as_int();
    double target_yaw_x_axis = this->get_parameter("target_yaw_x_axis").as_double();
    if (green_light_->is_detected == true)
    {
      double yaw_angle_calibration_factor = this->get_parameter("yaw_angle_calibration_factor").as_double();
      // 计算偏移量
      double delta_x = green_light_->location.x - target_yaw_x_axis;
      // 限制delta_x在+-5000内
      if (delta_x > MAX_YAW_CALIBRATION_DELTA_X)
        delta_x = 5000;
      else if (delta_x < -MAX_YAW_CALIBRATION_DELTA_X)
        delta_x = -5000;
      // 区间判断，若偏移量为正说明绿灯在右侧，应该向左转，调小target_yaw_angle
      if (delta_x > 5)
      {
        target_yaw_angle -= delta_x * yaw_angle_calibration_factor;
      }
      else if (delta_x < -5)
      {
        target_yaw_angle -= delta_x * yaw_angle_calibration_factor;
      }

      if (target_yaw_angle > YAW_MAX_ANGLE)
        target_yaw_angle = YAW_MAX_ANGLE;
      else if (target_yaw_angle < 5000)
        target_yaw_angle = 5000;

      RCLCPP_INFO(this->get_logger(), "Calibrating yaw angle to %d", target_yaw_angle);
      lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle, target_yaw_angle); // 会自动调用回调函数进行更新
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Dart launcher is offline or in boot/protect state, skip calibration");
  }
}

rcl_interfaces::msg::SetParametersResult NodeLVGLUI::set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  RCLCPP_INFO(this->get_logger(), "Parameters set request received");
  if (!callback_disabled)
    update_parameters_to_gui();
  return result;
}

void NodeLVGLUI::update_green_light_callback(info::msg::GreenLight::SharedPtr msg)
{
  if (msg == nullptr)
    return;

  double last_location_x;
  if (green_light_)
    last_location_x = green_light_->location.x;
  else
    last_location_x = msg->location.x;
  // 低通滤波
  msg->location.x = msg->location.x * this->get_parameter("yaw_angle_calibration_filter_factor").as_double() + last_location_x * (1 - this->get_parameter("yaw_angle_calibration_filter_factor").as_double());
  if (!msg->is_detected)
    msg->location.x = last_location_x;
  green_light_ = msg;

  with_ui_lock(node, [&]()
               {
    if (msg->is_detected)
    {
      static char buf[30];
      sprintf(buf, "%.1f, %.1f", msg->location.x, msg->location.y);
      RCLCPP_INFO(this->get_logger(), "Green light detected at %.1f, %.1f", msg->location.x, msg->location.y);
      lv_label_set_text(guider_ui.Main_label_yaw_location, buf);
      lv_label_set_text(guider_ui.Main_label_yaw_location_cv, buf);
    }
    else
    {
      lv_label_set_text(guider_ui.Main_label_yaw_location, "N/A");
      lv_label_set_text(guider_ui.Main_label_yaw_location_cv, "N/A");
    } });

  if (this->get_parameter("auto_yaw_calibration").as_bool())
    calibration_yaw(); // 10Hz
}

void NodeLVGLUI::update_dart_launcher_status_callback(info::msg::DartLauncherStatus::SharedPtr msg)
{
  // 离线警告 如果bool任意一个为零则将msgbox取消hidden，内容为"弹鼓/Yaw轴/丝杆/摩擦轮电机/遥控器/C板/裁判离线"，如多个则用 连接并叠加
  /*
  bool motor_ls_online 0
  bool motor_y_online 0
  bool motor_dm_online 0
  bool judge_online 0
  bool[4] motor_fw_online [0,0,0,0]
  bool rc_online 0
  bool dart_launcher_online 0
  */
  if (msg == nullptr)
    return;

  dart_launcher_status_ = msg;

  if (!mutex_ui_.try_lock())
    return;

  std::string msgbox_text = "";
  if (!msg->motor_ls_online)
    msgbox_text += "丝杆 ";
  if (!msg->motor_y_online)
    msgbox_text += "Yaw轴 ";
  if (!msg->motor_dm_online)
    msgbox_text += "弹鼓 ";
  if (!msg->judge_online)
    msgbox_text += "裁判 ";
  if (!msg->rc_online)
    msgbox_text += "遥控器 ";

  if (!msg->motor_fw_online[0])
    msgbox_text += "摩擦轮1 ";
  if (!msg->motor_fw_online[1])
    msgbox_text += "摩擦轮2 ";
  if (!msg->motor_fw_online[2])
    msgbox_text += "摩擦轮3 ";
  if (!msg->motor_fw_online[3])
    msgbox_text += "摩擦轮4 ";
  if (!msg->dart_launcher_online)
    msgbox_text = "C板 ";

  if (msgbox_text != "")
  {
    msgbox_text += "离线  ";
    lv_obj_clear_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_HIDDEN);
    if (msgbox_text != lv_label_get_text(lv_msgbox_get_text(guider_ui.Main_msgbox)))
    {
      lv_label_set_text(lv_msgbox_get_title(guider_ui.Main_msgbox), "警告");
      lv_label_set_text(lv_msgbox_get_text(guider_ui.Main_msgbox), msgbox_text.c_str());
    }
  }
  else
  {
    lv_obj_add_flag(guider_ui.Main_msgbox, LV_OBJ_FLAG_HIDDEN);
  }

  // uint8 dart_state # boot = 100, protect = 101, remote = 102, match = 103-106 Enter Wait Launch Reload, undefined=255

  if (!msg->dart_launcher_online)
    lv_label_set_text(guider_ui.Main_label_state, "Unknown");
  else if (msg->dart_state >= 103 && msg->dart_state <= 106)
  {
    lv_label_set_text(guider_ui.Main_label_state, "Match");
    switch (msg->dart_state)
    {
    case 103:
      lv_label_set_text(guider_ui.Main_label_23, "复位中");
      break;
    case 104:
      lv_label_set_text(guider_ui.Main_label_23, "等待中");
      break;
    case 105:
      lv_label_set_text(guider_ui.Main_label_23, "正在推出");
      break;
    case 106:
      lv_label_set_text(guider_ui.Main_label_23, "装填中");
      break;
    }
    lv_label_set_text_fmt(guider_ui.Main_label_launch_progress, "%d/4", msg->dart_launch_process);
    lv_bar_set_value(guider_ui.Main_bar_launch_progress, msg->dart_launch_process * 25, LV_ANIM_OFF);
  }
  else
  {
    if (msg->motor_dm_resetting || msg->motor_ls_resetting || msg->motor_y_resetting)
    {
      lv_label_set_text(guider_ui.Main_label_23, "复位中");
    }
    else
    {
      lv_label_set_text(guider_ui.Main_label_23, "不适用");
    }

    if (msg->dart_state == 100)
      lv_label_set_text(guider_ui.Main_label_state, "Boot");
    else if (msg->dart_state == 101)
      lv_label_set_text(guider_ui.Main_label_state, "Protect");
    else if (msg->dart_state == 102)
      lv_label_set_text(guider_ui.Main_label_state, "Remote");
    else
      lv_label_set_text(guider_ui.Main_label_state, "Unknown");

    lv_label_set_text(guider_ui.Main_label_launch_progress, "0/4");
    lv_bar_set_value(guider_ui.Main_bar_launch_progress, 0, LV_ANIM_OFF);
  }

  // float32 voltage[4] 取平均
  double voltage = 0;
  int online_count[2];
  online_count[0] = msg->motor_fw_online[0] + msg->motor_fw_online[1];
  online_count[1] = msg->motor_fw_online[2] + msg->motor_fw_online[3];
  if (online_count[0] + online_count[1] > 0)
  {
    for (int i = 0; i < 4; i++)
      voltage += msg->bus_voltage[i];
    voltage /= online_count[0] + online_count[1];
  }
  char voltage_str[10];
  sprintf(voltage_str, "%.1fV", voltage);
  if (voltage == 0)
    lv_label_set_text(guider_ui.Main_label_voltage, "N/A");
  else
    lv_label_set_text(guider_ui.Main_label_voltage, voltage_str);

  // 摩擦轮速度
  int32_t motor_fw_velocity_average[] = {0, 0};
  if (online_count[0] != 0)
    motor_fw_velocity_average[0] = (msg->motor_fw_velocity[0] + msg->motor_fw_velocity[1]) / online_count[0];
  if (online_count[1] != 0)
    motor_fw_velocity_average[1] = (msg->motor_fw_velocity[2] + msg->motor_fw_velocity[3]) / online_count[1];
  lv_label_set_text_fmt(guider_ui.Main_label_fw_speed_1, "%d", motor_fw_velocity_average[0]);
  lv_label_set_text_fmt(guider_ui.Main_label_fw_speed_2, "%d", motor_fw_velocity_average[1]);
  lv_meter_set_indicator_value(guider_ui.Main_meter_fw_speed_1, guider_ui.Main_meter_fw_speed_1_scale_0_ndline_0, motor_fw_velocity_average[0]);
  lv_meter_set_indicator_value(guider_ui.Main_meter_fw_speed_2, guider_ui.Main_meter_fw_speed_2_scale_0_ndline_0, motor_fw_velocity_average[1]);

  // Yaw轴角度
  lv_label_set_text_fmt(guider_ui.Main_label_yaw_angle, "%d", msg->motor_y_angle);
  lv_bar_set_value(guider_ui.Main_bar_yaw_angle, (msg->motor_y_angle / YAW_MAX_ANGLE) * 100, LV_ANIM_OFF);
  mutex_ui_.unlock();
}

void NodeLVGLUI::update_cv_image(sensor_msgs::msg::Image::SharedPtr msg)
{
  static lv_color_t img_buf_[2][LV_CANVAS_BUF_SIZE_TRUE_COLOR(606, 485)];
  static int buf_index = 0;
  buf_index %= 2;
  // img_buf = img_buf_;
  if (msg == nullptr)
    return;
  // 图像是bgr8
  // 用cv_bridge压缩到606*485
  cv::Mat img_resized =
      cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

  // 压缩
  cv::resize(img_resized, img_resized, cv::Size(606, 485), 0, 0, cv::INTER_LINEAR);

  // 转换为16位加载到lvgl buf
  for (int row = 0; row < 485; ++row)
  {
    for (int col = 0; col < 606; ++col)
    {
      auto px = img_resized.at<cv::Vec3b>(row, col);
      img_buf_[buf_index][row * 606 + col] = LV_COLOR_MAKE(px[2], px[1], px[0]);
    }
  }
  mutex_ui_.lock();
  // lv_canvas_copy_buf(guider_ui.Main_canvas_opencv, img_buf_, 0, 0, 606, 485); // 双缓冲绘制
  lv_canvas_set_buffer(guider_ui.Main_canvas_opencv, img_buf_[buf_index], 606, 485, LV_IMG_CF_TRUE_COLOR);
  lv_obj_invalidate(guider_ui.Main_canvas_opencv);
  mutex_ui_.unlock();
  buf_index++;
}

void NodeLVGLUI::update_parameters_to_gui()
{
  mutex_ui_.lock();
  // 禁用回调
  callback_disabled = true;

  lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle, this->get_parameter("target_yaw_angle").as_int());
  lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_offset, this->get_parameter("target_yaw_angle_offset").as_int());
  lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle_cv, this->get_parameter("target_yaw_angle").as_int());
  lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_angle_offset_cv, this->get_parameter("target_yaw_angle_offset").as_int());
  lv_spinbox_set_value(guider_ui.Main_spinbox_fw_speed, this->get_parameter("target_fw_velocity").as_int());
  lv_spinbox_set_value(guider_ui.Main_spinbox_fw_speed_offset, this->get_parameter("target_fw_velocity_offset").as_int());
  lv_spinbox_set_value(guider_ui.Main_spinbox_fw_speed_ratio, this->get_parameter("target_fw_velocity_ratio").as_double() * 1000);
  lv_spinbox_set_value(guider_ui.Main_spinbox_yaw_calibration_x, this->get_parameter("target_yaw_x_axis").as_double() * 100);
  // Switch value
  set_switch_state(guider_ui.Main_sw_auto_rpm_calibration, this->get_parameter("auto_fw_calibration").as_bool());
  set_switch_state(guider_ui.Main_sw_auto_yaw_calibration, this->get_parameter("auto_yaw_calibration").as_bool());
  set_switch_state(guider_ui.Main_sw_auto_yaw_calibration_cv, this->get_parameter("auto_yaw_calibration").as_bool());

  callback_disabled = false;
  mutex_ui_.unlock();
}

void NodeLVGLUI::update_dart_launcher_present_param_callback(info::msg::DartParam::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received dart_launcher_present_param");
  loadParametersfromMsg(*this, msg);
  update_parameters_to_gui();
}

void NodeLVGLUI::update_ip_address()
{
  // 获取IP地址
  std::string interface = "wlan0";
  std::string ip = getIPAddress(interface);

  if (!mutex_ui_.try_lock())
    return;
  RCLCPP_INFO_ONCE(this->get_logger(), "IP address: %s", ip.c_str());
  lv_label_set_text(guider_ui.Main_label_ip, ip.c_str());
  mutex_ui_.unlock();
}

int main(void)
{
  // ROS 2 Interface begin
  rclcpp::init(0, nullptr);
  node = std::make_shared<NodeLVGLUI>();
  rclcpp::spin(node);
  RCLCPP_INFO(node->get_logger(), "UI thread stopped");

  rclcpp::shutdown();
  return true;
}