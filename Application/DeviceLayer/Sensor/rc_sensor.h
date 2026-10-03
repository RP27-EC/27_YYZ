#ifndef DOWN_RC_SENSOR_H
#define DOWN_RC_SENSOR_H
#include <stdint.h>
typedef struct { uint8_t value; } remote_switch_info_t;
typedef struct {
    int16_t ch0, ch1, ch2, ch3;
    remote_switch_info_t s1, s2;
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
int RC_Sensor_Online(uint32_t now_ms);
#endif
