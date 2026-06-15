#include "can.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/*
 * CAN frame parser
 *
 * Supports the standard candump log format:
 *   vcan0  18FEF100   [8]  00 00 7D 00 00 00 00 00
 */

bool can_parse_line(const char *line, can_frame_t *frame) {
    if (!line || !frame) return false;

    /* Skip leading whitespace and empty lines */
    while (*line && isspace((unsigned char)*line)) line++;
    if (*line == '\0' || *line == '#') return false;

    /* Skip the interface name (first token) */
    char iface[32];
    char id_str[16];
    int  dlc;

    /* sscanf: interface  ID  [DLC]  then up to 8 data bytes */
    if (sscanf(line, "%31s %15s [%d]", iface, id_str, &dlc) != 3) return false;
    if (dlc < 0 || dlc > CAN_MAX_DLC) return false;

    /* Parse the hex ID */
    char *end;
    uint32_t id = (uint32_t)strtoul(id_str, &end, 16);
    if (end == id_str) return false; /* no digits consumed */

    frame->id       = id;
    frame->extended = (strlen(id_str) > 3 || id > 0x7FF);
    frame->dlc      = (uint8_t)dlc;
    memset(frame->data, 0, CAN_MAX_DLC);

    /* Advance past "[DLC]" to the data bytes */
    const char *p = strstr(line, "]");
    if (!p) return false;
    p++; /* skip ] */

    for (int i = 0; i < dlc; i++) {
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p == '\0') return false;
        frame->data[i] = (uint8_t)strtoul(p, &end, 16);
        if (end == p) return false;
        p = end;
    }

    return true;
}

void can_print_frame(const can_frame_t *frame) {
    printf("  ID : 0x%08X (%s)\n",
           frame->id,
           frame->extended ? "29-bit extended" : "11-bit standard");
    printf("  DLC: %d bytes\n", frame->dlc);
    printf("  Data:");
    for (int i = 0; i < frame->dlc; i++) printf(" %02X", frame->data[i]);
    printf("\n");
}
