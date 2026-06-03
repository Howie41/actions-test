/**
 * @file arm_task.cpp
 * @author FunFer
 * @brief 取矿任务实现
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "arm_task.hpp"

#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "logger.hpp"


osThreadId_t Arm_TaskHandle;

extern Logger logger;

static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};


extern Arm arm;

uint8_t flag = 1;

float tar_pos_3508 = 0.0f;
float tar_pos_2006 = 0.0f;
float tar_pos_4340 = 0.0f;
float tar_pos_4310 = 0.0f;
uint8_t fetch_flag = 0;

void armTask(void *argument) {
    
  for (;;) {

    if (arm_cmd_sub.TryGet(&arm_cmd)) {
        if (arm_cmd.update) {
            arm.step_fetch(1, flag);
            flag++;
            if (flag == 7) flag = 1;
        }
    }
    // if (arm_cmd_sub.TryGet(&arm_cmd)) {
    //     if (arm_cmd.update) {
    //         flag = 1;
    //     }
    //     if (arm_cmd.fetch) {
    //         if (fetch_flag) {
    //             fetch_flag = 0;
    //             arm.release();
    //         } else {
    //             fetch_flag = 1;
    //             arm.fetch();
    //         }
    //     }
    // }

    // if (flag) {
    //     flag = 0;
    //     arm.arm_lift_.posWithSpeedControl(tar_pos_4340, 1000.0f);
    //     arm.arm_rotate_.posWithSpeedControl(tar_pos_3508, 2.8f, 10.0f, 30.0f, 0.0f, 0.0f);
    //     arm.arm_expand_.posWithSpeedControl(tar_pos_2006, 12.0f, 180.0f, 360.0f, 0.0f, 0.0f);
    //     arm.arm_flip_.posWithSpeedControl(tar_pos_4310, 120.0f);
    // }

    osDelay(1);
  }
}
