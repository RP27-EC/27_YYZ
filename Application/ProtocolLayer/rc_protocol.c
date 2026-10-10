/**
 * @file rc_protocol.c
 * @brief 校验DBUS四通道、开关、键鼠并解析拨轮，发布完整快照。
 */
/* Includes ------------------------------------------------------------------*/
#include "rc_protocol.h"

/* Exported functions --------------------------------------------------------*/
/** @brief 将18字节DBUS帧解码为遥控快照。 */
int RC_DecodeDbus(const uint8_t *d, uint32_t size, rc_sensor_info_t *out)
{
    if (size != 18U) { return 0; }
    rc_sensor_info_t r = {0};
    r.ch0 = (int16_t)((d[0] | (d[1] << 8)) & 0x7ff) - 1024;
    r.ch1 = (int16_t)(((d[1] >> 3) | (d[2] << 5)) & 0x7ff) - 1024;
    r.ch2 = (int16_t)(((d[2] >> 6) | (d[3] << 2) | (d[4] << 10)) & 0x7ff) - 1024;
    r.ch3 = (int16_t)(((d[4] >> 1) | (d[5] << 7)) & 0x7ff) - 1024;
    r.s1.value = (d[5] >> 6) & 3;
    r.s2.value = (d[5] >> 4) & 3;
    if (r.s1.value == 0 || r.s2.value == 0 || d[12] > 1 || d[13] > 1) { return 0; }
    const int16_t channels[4] = {r.ch0, r.ch1, r.ch2, r.ch3};
    for (unsigned i = 0; i < 4; ++i) {
        if (channels[i] < -660 || channels[i] > 660) { return 0; }
    }
    uint16_t wheel = (uint16_t)d[16] | (uint16_t)d[17] << 8;
    r.thumbwheel = wheel >= 364U && wheel <= 1684U ? (int16_t)wheel - 1024 : 0;
    r.mouse_vx = (int16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));
    r.mouse_vy = (int16_t)((uint16_t)d[8] | ((uint16_t)d[9] << 8));
    r.mouse_vz = (int16_t)((uint16_t)d[10] | ((uint16_t)d[11] << 8));
    r.mouse_btn_l.value = d[12]; r.mouse_btn_r.value = d[13];
    r.key_v = (uint16_t)d[14] | ((uint16_t)d[15] << 8);
    rc_button_info_t *keys[] = {&r.W, &r.S, &r.A, &r.D, &r.Shift, &r.Ctrl, &r.Q, &r.E,
                              &r.R, &r.F, &r.G, &r.Z, &r.X, &r.C, &r.V, &r.B};
    for (unsigned i = 0; i < 16; ++i) { keys[i]->value = (r.key_v >> i) & 1U; }
    /* Publish only a complete validated frame; preserve timestamp/counter. */
    r.valid = 1; r.frames = out->frames; r.last_ms = out->last_ms;
    *out = r;
    return 1;
}
