/**
 * @file    yaw_probe_config.h
 * @brief   Yaw电机只读反馈探测的CAN编号与时序配置。
 */
#ifndef DOWN_YAW_PROBE_CONFIG_H
#define DOWN_YAW_PROBE_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
#define YAW_PROBE_CAN_ID 0x142U             /**< 参考工程KT9025的CAN1发送及反馈ID。 */
#define YAW_PROBE_STATE1 0x9AU              /**< 只读电压、温度和错误状态。 */
#define YAW_PROBE_STATE2 0x9CU              /**< 只读编码器、速度、温度和电流。 */
#define YAW_PROBE_QUERY_MS 50U              /**< 两次已入队查询之间的最短间隔，毫秒。 */
#define YAW_PROBE_OFFLINE_MS 250U           /**< 状态2反馈超时阈值，毫秒。 */
#define YAW_PROBE_TX_TIMEOUT_MS 20U         /**< 查询发送请求超时取消阈值，毫秒。 */
#define YAW_PROBE_STATE1_DIVIDER 5U         /**< 每五次查询中一次状态1，其余状态2。 */
#define YAW_PROBE_ENCODER_COUNTS 65536L     /**< 编码器一圈原始计数，按参考协议解释。 */
#define YAW_PROBE_HALF_COUNTS 32768L        /**< 半圈计数，用于最短角差和回绕判断。 */
#define YAW_PROBE_ZERO_ENCODER 58768U       /**< 正前读数0xE590，暂定机械零点。 */
#define YAW_PROBE_DIRECTION 1               /**< 当前观察为俯视逆时针增大，暂取+1。 */

#endif
