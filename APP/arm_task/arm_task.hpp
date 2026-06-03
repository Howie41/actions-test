/**
 * @file arm_task.hpp
 * @author FunFer
 * @brief 取矿机构任务
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 * @note :
 * @versioninfo :
 */

#pragma once

#include "Motor.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include <cmath>
#include <stdint.h>
#include <stdio.h>


class Arm {

public:
    Arm(DM43xxMotor &arm_lift, MotorBase &arm_rotate, MotorBase &arm_expand, DM43xxMotor &arm_flip) : arm_lift_(arm_lift), arm_rotate_(arm_rotate), arm_expand_(arm_expand), arm_flip_(arm_flip) {}
    ~Arm() {}

    Arm& fetch() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
        return *this;
    }
    Arm& release() {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
        return *this;
    }

    Arm& setHeight(float height_rate) {
        arm_lift_.posWithSpeedControl(height_rate * 1080.0f, 1000.0f);
        return *this;
    }

    Arm& setRotate(float rotate_angle) {
        arm_rotate_.posWithSpeedControl(rotate_angle, 10.0f, 30.0f, 0.0f, 0.0f);
        return *this;
    }

    Arm& setExpand(float expand_rate) {
        arm_expand_.posWithSpeedControl(expand_rate * 1080.0f, 8.0f, 0.2f, 0.3f, 0.0f, 0.0f);
        return *this;
    }

    Arm& setFlip(float flip_angle) {
        arm_flip_.posWithSpeedControl(flip_angle, 120.0f);
        return *this;
    }

    bool getIsFinished() {
        return arm_expand_.getIsFinished() && arm_rotate_.getIsFinished();
    }
    Arm& setAngle(float a, float b, float c, float d) {
        arm_lift_.posWithSpeedControl(a, 1000.0f);
        arm_rotate_.posWithSpeedControl(b, 2.8f, 10.0f, 30.0f, 0.0f, 0.0f);
        arm_expand_.posWithSpeedControl(c, 20.0f, 180.0f, 360.0f, 0.0f, 0.0f);
        arm_flip_.posWithSpeedControl(d, 120.0f);
        return *this;
    }
    Arm& reset() {
        setAngle(0.0f, 0.0f, 0.0f, 0.0f);
        return *this;
    }
    Arm& step_fetch(int8_t step, uint8_t index) {
        switch (step) {
            case 1:
                switch (index) {
                    case 1:
                        setAngle(0.0f, -10.0f, 1000.0f, 85.0f);
                        break;
                    case 2:
                        setAngle(0.0f, 6.0f, 1000.0f, 85.0f);
                        fetch();
                        break;
                    case 3:
                        setAngle(540.0f, -30.0f, 400.0f, 85.0f);
                        break;
                    case 4:
                        setAngle(540.0f, -90.0f, 400.0f, -66.0f);
                        break;
                    case 5:
                        release();
                        break;
                    case 6:
                        reset();
                        break;
                }
                break;
            case 2:
                break;
            case -1:
                break;
        }
        return *this;
    }

    DM43xxMotor &arm_lift_;
    MotorBase &arm_rotate_;
    MotorBase &arm_expand_;
    DM43xxMotor &arm_flip_;

};

void armTask(void *argument);
