#include "rc_sensor.h"
#include "rc_protocol.h"
#include "chassis_config.h"
rc_sensor_info_t rc_sensor_info;
rc_sensor_t rc_sensor = {.info = &rc_sensor_info, .init = RC_Sensor_Init, .update = RC_Sensor_Update};
volatile rc_receive_status_t rc_receive_status;
static volatile rc_sensor_info_t received_remote;
void RC_Sensor_Init(rc_sensor_t *sensor)
{
    *sensor->info = (rc_sensor_info_t){0};
    received_remote = (rc_sensor_info_t){0};
    rc_receive_status = (rc_receive_status_t){0};
}
void RC_Sensor_Update(rc_sensor_t *sensor) { *sensor->info = received_remote; }
void RC_Sensor_Invalidate(void)
{
    received_remote.valid = 0;
    rc_receive_status.good_streak = 0;
}
void RC_Sensor_Receive(const uint8_t *data, uint32_t size, uint32_t now_ms)
{
    rc_sensor_info_t next = received_remote;
    if (!RC_DecodeDbus(data, size, &next)) {
        ++rc_receive_status.bad_frames;
        RC_Sensor_Invalidate();
        return;
    }
    ++next.frames;
    next.last_ms = now_ms;
    if (rc_receive_status.good_streak < RC_VALID_FRAME_COUNT) { ++rc_receive_status.good_streak; }
    next.valid = rc_receive_status.good_streak >= RC_VALID_FRAME_COUNT;
    received_remote = next;
}
int RC_Sensor_Online(uint32_t now_ms)
{
    return rc_sensor.info->valid && rc_sensor.info->frames &&
           (uint32_t)(now_ms - rc_sensor.info->last_ms) <= CHASSIS_OFFLINE_MS;
}
