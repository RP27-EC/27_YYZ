/**
 * @file    kt_yaw_protocol.h
 * @brief   KT类Yaw只读查询、受限电流帧与状态解码接口。
 */
#ifndef DOWN_KT_YAW_PROTOCOL_H
#define DOWN_KT_YAW_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported typedef ----------------------------------------------------------*/
typedef struct {
    uint16_t encoder;       /**< 单圈编码器原始计数，尚未标定机械零点。 */
    int16_t speed_dps;      /**< 按参考协议解释的角速度，度/秒。 */
    int16_t current_raw;    /**< 电流原始反馈，暂不换算安培。 */
    int8_t temperature_c;   /**< 温度反馈，摄氏度。 */
    uint16_t voltage_raw;   /**< 电压原始反馈，型号确认前不换算伏特。 */
    uint8_t error_state;    /**< 状态1的原始错误位。 */
} kt_yaw_status_t;

/* Exported functions --------------------------------------------------------*/
/** @brief 构造8字节只读查询，仅允许0x9A和0x9C；成功返回1。 */
int KT_Yaw_BuildRead(uint8_t command, uint8_t out[8]);
/** @brief 解码8字节状态1/2和A1回复，保留其他字段，返回命令字或0。 */
uint8_t KT_Yaw_DecodeStatus(const uint8_t *data, uint32_t size, kt_yaw_status_t *status);
/** @brief 构造独立0xA1电流帧，拒绝超出首轮输出限制的原始值。 */
int KT_Yaw_BuildCurrent(int16_t current_raw, uint8_t out[8]);

#endif
