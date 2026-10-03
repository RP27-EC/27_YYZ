#ifndef DOWN_RC_PROTOCOL_H
#define DOWN_RC_PROTOCOL_H
#include "rc_sensor.h"
int RC_DecodeDbus(const uint8_t *data, uint32_t size, rc_sensor_info_t *out);
#endif
