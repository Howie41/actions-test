/**
 * @file infrared_com.hpp
 * @author zhy (Howie41)
 * @brief 红外通信模块
 * @date 2026-05-16
 *
 * @note 模块使用了CMSIS-OS API，务必在RTOS内核后启动后再发送数据！
 * @note 模块波特率默认为9600bps
 */

#pragma once
#include "cmsis_os2.h"
#include "main.h"
#include "UartPort.hpp"
#include "stm32h7xx_hal_uart.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "usart.h"

#include <cstddef>
#include <cstdint>
#include <atomic>

class InfraredModule {
    public:
        static constexpr uint32_t TX_MAX_TIMEOUT = 1000;
        static constexpr const char *INFRARED_MSG_TOPIC = "infrared_msg";

        explicit InfraredModule(UartPort &uart_port) : uart_port_(uart_port) {}
        ~InfraredModule() = default;

        /**
         * @brief 由 UartPort 的接收回调转发进来
         * @param data 新收到的数据片段
         * @param len 数据长度
         */
        static constexpr uint8_t HEADER = 0xAA;
        static constexpr uint8_t FOOTER = 0xBB;
        static constexpr uint8_t RAW_LENGTH = 5; // 包括头尾的总长度
        void UartPortRxCbHandler(const uint8_t *data, size_t len) {
            if (data == nullptr || len == 0) return;
                if (len < RAW_LENGTH) return;
                for (size_t i = 0; i <= len - RAW_LENGTH; ++i) {
                    // 0xAA 0xXX 0xXX 0xXX 0xBB
                    if (data[i] == HEADER && data[i + RAW_LENGTH - 1] == FOOTER
                        && data[i + 1] == data[i + 2] && data[i + 2] == data[i + 3]) {
                        pub_infrared_msg msg{};
                        msg.data = data[i + 1];
                        infrared_pub_.Publish(msg);
                    }
                    break; // 只处理一个有效包
                }
        }

    private:
        static constexpr uint8_t ACK_CODE = 0xF1;

        UartPort &uart_port_;

        TypedTopicPublisher<pub_infrared_msg> infrared_pub_{INFRARED_MSG_TOPIC};
        pub_infrared_msg infrared_msg_{};
};