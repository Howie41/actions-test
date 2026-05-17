#ifndef TAIL_CLAW_TASK_HPP
#define TAIL_CLAW_TASK_HPP 

#include "com_config.h"
#include "pid_controller.h"

#include "task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include <cstdint>

typedef enum weapon_match_state{
    none=0x00,                             //0000 0000什么都不用调整
    chassis_clockwise_rotation=0x01,       //0000 0001底盘顺时针转
    chassis_contrarotate=0x02,             //0000 0010底盘逆时针转
    motor_move_left=0x04,                  //0000 0100左动
    motor_move_right=0x08,                 //0000 1000右动
    motor_roll_up=0x10,                    //0001 0000上转
    motor_roll_down=0x20,                  //0010 0000下转
    ismatch=0x40,                          //0100 0000武器已经匹配
}weapon_match_state;

void tail_claw_init(void);
void tail_claw_task(void *argument);

void weapon_open(bool open);
void KFS_open(bool open);

float set_move_pos(float pos,PID_t *pos_pid,PID_t *speed_pid);
float set_roll_pos(float pos,PID_t *pos_pid,PID_t *speed_pid);

void get_weapon_match_state();
void tail_claw_move_close();

#endif