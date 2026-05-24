/**
 * @file motor_task.cpp
 * @author FunFer
 * @brief
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "stm32h7xx_hal.h"
#include "motor_task.h"
#include "com_config.h"
#include "Motor.hpp"
#include "pid_controller.h"
#include "topics.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <vector>


osThreadId_t Motor_TaskHandle;


extern C620Motor arm3508_motor;
extern C610Motor arm2006_motor;


struct MotorPlanningUnit {
    MotorBase *motor;
    PID_t pos_pid;
    PID_t speed_pid;
};


class MotorPlanningSystem {
public:
    void registerMotor(MotorBase &motor) {
        motor_planning_units_.push_back(MotorPlanningUnit{
            .motor = &motor,
            .pos_pid = {
                .Kp = 45.0f,
                .Ki = 0.0f,
                .Kd = 2.5f,
                .MaxOut = 60.0f,
                .DeadBand = 0.1f
            },
            .speed_pid = {
                .Kp = 2000.0f,
                .Ki = 0.06f,
                .Kd = 1.8f,
                .MaxOut = 12000.0f,
                .DeadBand = 0.5f
            }
        });

        PID_Init(&motor_planning_units_.back().pos_pid);
        PID_Init(&motor_planning_units_.back().speed_pid);
    }

    void update() {
        for (auto &unit : motor_planning_units_) {
            unit.pos_pid.MaxOut = unit.motor->updatePosProcess();
            unit.motor->setMotorCmd(PID_Calculate(&unit.speed_pid, unit.motor->getCurrentSpeed(), PID_Calculate(&unit.pos_pid, unit.motor->getCurrentSumPos(), unit.motor->tar_sum_pos_)));
        }
    }
private:
    std::vector<MotorPlanningUnit> motor_planning_units_;

};


void motorTask(void *argument) {
  TickType_t currentTime;
  currentTime = xTaskGetTickCount();

    MotorPlanningSystem motor_planning_system;

    motor_planning_system.registerMotor(arm3508_motor);
    motor_planning_system.registerMotor(arm2006_motor);

  for (;;) {
    motor_planning_system.update();
    vTaskDelayUntil(&currentTime, 1);

  }
}



// static PID_t arm3508_pos_pid{
//     .Kp = 45.0f,
//     .Ki = 0.0f,
//     .Kd = 2.5f,
//     .MaxOut = 60.0f,
//     .DeadBand = 0.1f
// };
// static PID_t arm3508_speed_pid{
//     .Kp = 2000.0f,
//     .Ki = 0.06f,
//     .Kd = 1.8f,
//     .MaxOut = 12000.0f,
//     .DeadBand = 0.5f
// };
