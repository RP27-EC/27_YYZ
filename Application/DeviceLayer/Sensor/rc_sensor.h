/**
 * @file rc_sensor.h
 * @brief DBUS遥控、键鼠和拨轮快照及在线状态接口。
 */
#ifndef DOWN_RC_SENSOR_H
#define DOWN_RC_SENSOR_H
#include <stdint.h>
typedef struct { uint8_t value; } remote_switch_info_t;
typedef struct { uint8_t value; } rc_button_info_t;
enum {
    RC_KEY_W = 1U << 0, RC_KEY_S = 1U << 1, RC_KEY_A = 1U << 2, RC_KEY_D = 1U << 3,
    RC_KEY_SHIFT = 1U << 4, RC_KEY_CTRL = 1U << 5, RC_KEY_Q = 1U << 6, RC_KEY_E = 1U << 7,
    RC_KEY_R = 1U << 8, RC_KEY_F = 1U << 9, RC_KEY_G = 1U << 10, RC_KEY_Z = 1U << 11,
    RC_KEY_X = 1U << 12, RC_KEY_C = 1U << 13, RC_KEY_V = 1U << 14, RC_KEY_B = 1U << 15,
    RC_KEY_MOVEMENT = RC_KEY_W | RC_KEY_S | RC_KEY_A | RC_KEY_D
};
typedef struct {
    int16_t thumbwheel; /**< DBUS拨轮中心0，原始值；负端返回中心为UP事件。 */
    int16_t ch0, ch1, ch2, ch3;
    remote_switch_info_t s1, s2;
    int16_t mouse_vx, mouse_vy, mouse_vz;
    float mouse_x, mouse_y; /**< 10帧均值鼠标输入，DBUS原始标度。 */
    uint16_t key_v;
    rc_button_info_t mouse_btn_l, mouse_btn_r;
    rc_button_info_t W, S, A, D, Shift, Ctrl, Q, E, R, F, G, Z, X, C, V, B;
    uint8_t valid;
    uint32_t frames, last_ms;
} rc_sensor_info_t;
typedef struct rc_sensor_t {
    rc_sensor_info_t *info;
    void (*init)(struct rc_sensor_t *);
    void (*update)(struct rc_sensor_t *);
} rc_sensor_t;
typedef struct { uint32_t bad_frames, good_streak; } rc_receive_status_t;
extern rc_sensor_info_t rc_sensor_info;
extern rc_sensor_t rc_sensor;
extern volatile rc_receive_status_t rc_receive_status;
void RC_Sensor_Init(rc_sensor_t *sensor);
/* Snapshot together with Motor_Update() while interrupts are masked by the task. */
void RC_Sensor_Update(rc_sensor_t *sensor);
void RC_Sensor_Receive(const uint8_t *data, uint32_t size, uint32_t now_ms);
void RC_Sensor_Invalidate(void);
/** @brief 清除旧会话的通道、拨杆、按键和鼠标历史；调用时屏蔽接收中断。 */
void RC_Sensor_StandbyReset(void);
int RC_Sensor_Online(uint32_t now_ms);
#endif
