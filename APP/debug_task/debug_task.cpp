/**
 * @file debug_task.cpp
 * @brief 调试任务 — Firewater 实时绘图: laser1 测距
 */
#include "debug_task.h"
#include "task.h"
#include "cmsis_os2.h"
#include "logger.hpp"
#include "com_config.h"

extern LaserMeasure laser1;
extern Logger logger;

osThreadId_t Debug_TaskHandle;

void debugTask(void *argument) {
  (void)argument;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, 20);

    auto &r = laser1.latestResult();
    logger.log("%d,%d,%u\n",
               r.distance_mm,
               r.valid ? 1 : 0,
               r.frame_count);
  }
}
