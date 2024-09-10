//
// Created by cheny on 24-9-10.
//

#ifndef DART_MCU_DARTMCU_NODE_H
#define DART_MCU_DARTMCU_NODE_H

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/time.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int64.h>
#include <buzzer.h>
#include "buzzer_examples.h"

void microros_node_task(void);

extern rcl_allocator_t allocator;
extern rcl_publisher_t publisher;
extern rcl_node_t node;
extern rclc_support_t support;
extern rcl_timer_t timer;
extern rclc_executor_t executor;
extern std_msgs__msg__Int64 msg;
extern bool micro_ros_init_successful;

#endif //DART_MCU_DARTMCU_NODE_H
