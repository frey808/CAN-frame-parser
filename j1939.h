#ifndef J1939_H
#define J1939_H

#include "can.h"

/*
 * J1939 builds on 29-bit extended CAN frames.
 *
 * The 29-bit identifier is laid out as:
 *
 *   Bits 28-26  Priority        (3 bits)
 *   Bit  25     Reserved        (1 bit)
 *   Bit  24     Data Page       (1 bit)
 *   Bits 23-16  PDU Format (PF) (8 bits)
 *   Bits 15-8   PDU Specific    (8 bits) — destination addr if PF < 240,
 *                                          group extension if PF >= 240
 *   Bits 7-0    Source Address  (8 bits)
 *
 * PGN = { Reserved, Data Page, PF, PS } when PF >= 0xF0 (peer-to-peer PDU2)
 *      = { Reserved, Data Page, PF, 0 } when PF <  0xF0 (PDU1, destination addr)
 */

typedef struct {
    uint8_t  priority;
    uint8_t  reserved;
    uint8_t  data_page;
    uint8_t  pdu_format;   /* PF */
    uint8_t  pdu_specific; /* PS: dest addr (PF<0xF0) or group ext (PF>=0xF0) */
    uint8_t  src_addr;
    uint32_t pgn;
} j1939_header_t;

/* Decode a 29-bit extended CAN ID into a J1939 header */
void j1939_decode_header(uint32_t can_id, j1939_header_t *hdr);

/* Look up a human-readable name for a PGN. Returns "Unknown PGN" if not found. */
const char *j1939_pgn_name(uint32_t pgn);

/* Decode and print a full J1939 frame including known SPNs */
void j1939_print_frame(const can_frame_t *frame);

#endif
