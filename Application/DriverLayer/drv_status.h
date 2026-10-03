#ifndef DOWN_DRV_STATUS_H
#define DOWN_DRV_STATUS_H
#include <stdint.h>
typedef struct {
    uint32_t init_ok, fdcan_clock_hz, uart_events, uart_errors, bad_frames;
    uint32_t tx_queued, tx_errors, can_bus_off, can_error_passive;
    uint32_t can_last_error, rc_good_streak;
} mec_io_status_t;
/* Keep existing Watch expressions for communication diagnostics. */
extern volatile mec_io_status_t mec_io;
#endif
