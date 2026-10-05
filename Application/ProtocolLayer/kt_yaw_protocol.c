/**
 * @file    kt_yaw_protocol.c
 * @brief   构造KT类只读查询、受限电流帧并解码状态，不写入电机参数。
 */

/* Includes ------------------------------------------------------------------*/
#include "kt_yaw_protocol.h"
#include "yaw_probe_config.h"
#include "gimbal_config.h"
#include <string.h>

/* Private functions ---------------------------------------------------------*/
/** @brief 按小端字节序读取16位原始值。 */
static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/* Exported functions --------------------------------------------------------*/
/** @brief 构造经过白名单检查的只读状态查询。 */
int KT_Yaw_BuildRead(uint8_t command, uint8_t out[8])
{
    if (!out || (command != YAW_PROBE_STATE1 && command != YAW_PROBE_STATE2)) { return 0; }
    memset(out, 0, 8);
    out[0] = command;
    return 1;
}

/** @brief 解码参考KT状态1/2及0xA1状态回复，失败不改变已知状态。 */
uint8_t KT_Yaw_DecodeStatus(const uint8_t *data, uint32_t size, kt_yaw_status_t *status)
{
    if (!data || !status || size != 8U) { return 0; }
    kt_yaw_status_t next = *status;
    if (data[0] == YAW_PROBE_STATE1) {
        next.temperature_c = (int8_t)data[1];
        next.voltage_raw = read_u16(&data[3]);
        next.error_state = data[7];
    } else if (data[0] == YAW_PROBE_STATE2 || data[0] == GIMBAL_YAW_TORQUE_COMMAND) {
        next.temperature_c = (int8_t)data[1];
        next.current_raw = (int16_t)read_u16(&data[2]);
        next.speed_dps = (int16_t)read_u16(&data[4]);
        next.encoder = read_u16(&data[6]);
    } else { return 0; }
    *status = next;
    return data[0];
}

/** @brief 仅生成有符号小端电流命令，其他参数字节全部清零。 */
int KT_Yaw_BuildCurrent(int16_t current, uint8_t out[8])
{
    if (!out || current < -GIMBAL_YAW_COMMAND_LIMIT || current > GIMBAL_YAW_COMMAND_LIMIT) { return 0; }
    memset(out, 0, 8);
    out[0] = GIMBAL_YAW_TORQUE_COMMAND;
    out[4] = (uint8_t)(uint16_t)current;
    out[5] = (uint8_t)((uint16_t)current >> 8);
    return 1;
}
