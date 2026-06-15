#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_MAX_DLC 8

typedef struct {
    uint32_t id;        /* 11-bit standard or 29-bit extended */
    bool     extended;  /* true = 29-bit J1939 frame */
    uint8_t  dlc;       /* data length code, 0-8 */
    uint8_t  data[CAN_MAX_DLC];
} can_frame_t;

/* Parse a raw candump log line into a can_frame_t.
   Returns true on success, false if the line is malformed.
   candump format: "  vcan0  18FEF100   [8]  00 00 7D 00 00 00 00 00" */
bool can_parse_line(const char *line, can_frame_t *frame);

void can_print_frame(const can_frame_t *frame);

#endif
