/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "cmsis_os2.h"
#include <atomic>
#include <cstdint>

#include "state_machine_task.h"
#include "NavProtocol.hpp"
#include "infrared_com.hpp"
#include "topic_pool.h"
#include "topics.hpp"
osThreadId_t StateMachineTaskHandle;

static std::atomic<RobotState> current_state{RobotState::begin};
static pub_infrared_msg latest_infrared_msg{};
TypedTopicSubscriber<pub_infrared_msg> infrared_sub(InfraredModule::INFRARED_MSG_TOPIC, 1);

/**
 * @brief 等待直到条件满足
 * @param condition 条件函数，传一个匿名函数就行，返回布尔值
 * @param delay_ms 多久检查一次条件
 */
template <typename T>
void wait_until(T &&condition, uint32_t delay_ms = 100U) {
    while (!condition()) {
        osDelay(delay_ms);
    }
}

/**
 * @brief 等待直到条件满足或超时
 * @param condition 条件函数，传一个匿名函数就行，返回布尔值
 * @param timeout_ms 超时时间
 * @param delay_ms 多久检查一次条件
 * @return true 条件满足，false 超时
 */
template <typename T>
bool wait_until_timeout_or(T &&condition, uint32_t timeout_ms, uint32_t delay_ms = 100U) {
    const uint32_t start = osKernelGetTickCount();
    while (!condition()) {
        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            return false;
        }
        osDelay(delay_ms);
    }
    return true;
}

void change_state_to(RobotState new_state) {
    current_state.store(new_state);
}

// 这个函数必须在任务环境里调用
void move_to_pos(int16_t x, int16_t y, int16_t yaw) {
    taskENTER_CRITICAL();
    nav_control::target_x = x;
    nav_control::target_y = y;
    nav_control::target_yaw = yaw;
    nav_control::arrived = false;
    taskEXIT_CRITICAL();
    wait_until([&]() { return nav_control::arrived; });
}

void stateMachineTask(void *argument) {
    for (;;) {
        switch (current_state.load()) {
        #ifdef MATCH_CWTY /** ========== 崇武探幽 单项赛 ========== */

            case RobotState::begin: {
                break;
            }

            case RobotState::go_to_SHR: {
                break;
            }

            case RobotState::aim_at_weapon: {
                break;
            }

            case RobotState::catch_weapon: {
                break;
            }

            case RobotState::rotate_weapon_claw: {
                break;
            }

            // R2松开武器头夹爪，等待操作手决策，决定是否拼装新的武器
            case RobotState::wait_for_cmd: {
                wait_until([&]() -> bool {
                    if (!infrared_sub.TryGet(&latest_infrared_msg)) return false;

                    switch (latest_infrared_msg.data) {
                        case 0x1A: // 夹取新的武器头
                            change_state_to(RobotState::go_to_SHR);
                            return true;
                        case 0x1B: // 进入梅林
                            change_state_to(RobotState::go_to_MF);
                            return true;
                        default:
                            return false;
                    }
                });
                break;
            }

        #elif MATCH_JGCB /** ========== 九宫藏宝 单项赛 ========== */


        #endif /** ============================================= */
            
            default: // 不应该到达这里
                break;

        }

        osDelay(1);
    }
}