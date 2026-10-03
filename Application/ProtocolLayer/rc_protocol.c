#include "rc_protocol.h"

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
    /* Publish only a complete validated frame; preserve timestamp/counter. */
    out->ch0 = r.ch0; out->ch1 = r.ch1; out->ch2 = r.ch2; out->ch3 = r.ch3;
    out->s1.value = r.s1.value;
    out->s2.value = r.s2.value;
    out->valid = 1;
    return 1;
}
