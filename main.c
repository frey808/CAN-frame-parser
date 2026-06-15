#include <stdio.h>
#include "can.h"

#define MAX_LINE 256
int main(void) {
    const char *samples[] = {
        /* EEC1 — engine at 1200 RPM, 50% torque, mode 3 */
        "vcan0  0CF00400   [8]  03 BD 32 60 09 FF FF FF",
        /* CCVS — 40 km/h, cruise off, park brake released */
        "vcan0  18FEF200   [8]  00 40 28 00 FF FF FF FF",
        /* ET1 — coolant 85°C, oil 95°C */
        "vcan0  18FEEE00   [8]  7D FF A0 2E FF FF FF FF",
        /* DM1 — one active DTC: SPN 100, FMI 4, OC 1 */
        "vcan0  18FECA00   [8]  AF FF C8 10 00 04 01 00",
        /* Proprietary A — manufacturer-specific */
        "vcan0  18FF0000   [8]  DE AD BE EF 01 02 03 04",
        /* Standard 11-bit frame (non-J1939) */
        "vcan0  123        [4]  AA BB CC DD",
        NULL
    };

    int frame_num = 0;
    for (int i = 0; samples[i] != NULL; i++) {
        can_frame_t frame;
        if (!can_parse_line(samples[i], &frame)) {
            printf("Parse error on sample %d\n", i);
            continue;
        }
        printf("Frame #%d  (raw: %s)\n", ++frame_num, samples[i]);
        can_print_frame(&frame);
        printf("\n");
    }

    return 0;
}
