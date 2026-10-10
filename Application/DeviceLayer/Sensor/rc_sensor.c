/**
 * @file rc_sensor.c
 * @brief 合法DBUS快照、在线恢复及鼠标10帧均值，失效时清除历史。
 */
/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"
#include "rc_protocol.h"
#include "chassis_config.h"
#include "gyro_config.h"
#include <string.h>
/* Private variables ---------------------------------------------------------*/
static int16_t mouse_samples[2][GYRO_MOUSE_FILTER_SAMPLES]; /**< 鼠标输入均值历史，原始值。 */
static uint8_t mouse_index; /**< 均值窗口下一写入位置。 */
/* Exported variables --------------------------------------------------------*/
rc_sensor_info_t rc_sensor_info;
rc_sensor_t rc_sensor = {.info = &rc_sensor_info, .init = RC_Sensor_Init, .update = RC_Sensor_Update};
volatile rc_receive_status_t rc_receive_status;
static volatile rc_sensor_info_t received_remote;
/* Exported functions --------------------------------------------------------*/
/** @brief 重置DBUS在线判断及鼠标滤波历史。 */
void RC_Sensor_Init(rc_sensor_t *sensor)
{
    *sensor->info = (rc_sensor_info_t){0};
    received_remote = (rc_sensor_info_t){0};
    rc_receive_status = (rc_receive_status_t){0};
    memset(mouse_samples, 0, sizeof(mouse_samples)); mouse_index = 0U;
}
/** @brief 将IRQ完成的快照复制到控制任务。 */
void RC_Sensor_Update(rc_sensor_t *sensor) { *sensor->info = received_remote; }
/** @brief 撤销遥控许可并清除历史鼠标输入。 */
void RC_Sensor_Invalidate(void)
{
    received_remote.valid = 0;
    received_remote.mouse_x = received_remote.mouse_y = 0.0f;
    memset(mouse_samples, 0, sizeof(mouse_samples)); mouse_index = 0U;
    rc_receive_status.good_streak = 0;
}
/** @brief 整车待命复位清除全部输入，保留帧计数、时刻及串口错误诊断。 */
void RC_Sensor_StandbyReset(void)
{
    RC_Sensor_Invalidate();
    received_remote = (rc_sensor_info_t){.frames = received_remote.frames, .last_ms = received_remote.last_ms};
}
/** @brief 只在合法新DBUS帧上推进恢复计数和鼠标滤波。 */
void RC_Sensor_Receive(const uint8_t *data, uint32_t size, uint32_t now_ms)
{
    rc_sensor_info_t next = received_remote;
    if (!RC_DecodeDbus(data, size, &next)) {
        ++rc_receive_status.bad_frames;
        return;
    }
    if (now_ms - received_remote.last_ms > CHASSIS_OFFLINE_MS) {
        memset(mouse_samples, 0, sizeof(mouse_samples)); mouse_index = 0U;
        rc_receive_status.good_streak = 0U;
    }
    mouse_samples[0][mouse_index] = next.mouse_vx;
    mouse_samples[1][mouse_index] = next.mouse_vy;
    mouse_index = (mouse_index + 1U) % GYRO_MOUSE_FILTER_SAMPLES;
    int32_t sum_x = 0, sum_y = 0;
    for (unsigned i = 0; i < GYRO_MOUSE_FILTER_SAMPLES; ++i) {
        sum_x += mouse_samples[0][i]; sum_y += mouse_samples[1][i];
    }
    next.mouse_x = (float)sum_x / GYRO_MOUSE_FILTER_SAMPLES;
    next.mouse_y = (float)sum_y / GYRO_MOUSE_FILTER_SAMPLES;
    ++next.frames;
    next.last_ms = now_ms;
    if (rc_receive_status.good_streak < RC_VALID_FRAME_COUNT) { ++rc_receive_status.good_streak; }
    next.valid = rc_receive_status.good_streak >= RC_VALID_FRAME_COUNT;
    received_remote = next;
}
/** @brief 判断合法恢复序列及最后完整帧的有效期。 */
int RC_Sensor_Online(uint32_t now_ms)
{
    return rc_sensor.info->valid && rc_sensor.info->frames &&
           (uint32_t)(now_ms - rc_sensor.info->last_ms) <= CHASSIS_OFFLINE_MS;
}
