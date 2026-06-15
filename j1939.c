#include "j1939.h"
#include <stdio.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Header decoding
 * ---------------------------------------------------------------------- */

void j1939_decode_header(uint32_t can_id, j1939_header_t *hdr) {
    hdr->src_addr    = (uint8_t)( can_id        & 0xFF);
    hdr->pdu_specific= (uint8_t)((can_id >>  8) & 0xFF);
    hdr->pdu_format  = (uint8_t)((can_id >> 16) & 0xFF);
    hdr->data_page   = (uint8_t)((can_id >> 24) & 0x01);
    hdr->reserved    = (uint8_t)((can_id >> 25) & 0x01);
    hdr->priority    = (uint8_t)((can_id >> 26) & 0x07);

    /* PGN includes DP + PF + PS when PF >= 0xF0 (broadcast PDU2).
       For PDU1 (PF < 0xF0), PS is the destination address, not part of PGN. */
    if (hdr->pdu_format >= 0xF0) {
        hdr->pgn = ((uint32_t)hdr->reserved   << 17)
                 | ((uint32_t)hdr->data_page   << 16)
                 | ((uint32_t)hdr->pdu_format  <<  8)
                 |  (uint32_t)hdr->pdu_specific;
    } else {
        hdr->pgn = ((uint32_t)hdr->reserved   << 17)
                 | ((uint32_t)hdr->data_page   << 16)
                 | ((uint32_t)hdr->pdu_format  <<  8);
    }
}

/* -------------------------------------------------------------------------
 * PGN name table — a small but representative set for heavy vehicle work
 * ---------------------------------------------------------------------- */

typedef struct { uint32_t pgn; const char *name; } pgn_entry_t;

static const pgn_entry_t pgn_table[] = {
    { 0xF004, "EEC1  — Electronic Engine Controller 1 (engine speed, torque)" },
    { 0xF005, "EEC2  — Electronic Engine Controller 2 (throttle, load)"       },
    { 0xFEF1, "EBC1  — Electronic Brake Controller 1"                          },
    { 0xFEF2, "CCVS  — Cruise Control / Vehicle Speed"                         },
    { 0xFEF6, "LFE   — Fuel Economy"                                           },
    { 0xFEF7, "VD    — Vehicle Distance"                                        },
    { 0xFECA, "DM1   — Active Diagnostic Trouble Codes"                        },
    { 0xFECB, "DM2   — Previously Active DTCs"                                 },
    { 0xFEDA, "SOC1  — Supplemental Engine/Transmission"                       },
    { 0xFF00, "Proprietary A — manufacturer-specific (broadcast)"              },
    { 0xEF00, "Proprietary B — manufacturer-specific (peer-to-peer)"           },
    { 0xFEEE, "ET1   — Engine Temperature 1 (coolant, oil temp)"               },
    { 0xFEF5, "EFL   — Engine Fluid Level/Pressure"                            },
    { 0xFEC1, "B_VP  — Battery / Voltage / Pressure"                           },
    { 0x0000, NULL }   /* sentinel */
};

const char *j1939_pgn_name(uint32_t pgn) {
    for (int i = 0; pgn_table[i].name != NULL; i++) {
        if (pgn_table[i].pgn == pgn) return pgn_table[i].name;
    }
    return "Unknown PGN";
}

/* -------------------------------------------------------------------------
 * SPN extraction helpers
 *
 * SPNs are packed into the 8 data bytes according to the J1939-71 spec.
 * Each PGN defines which bits carry which SPN.
 * We decode a representative subset here.
 * ---------------------------------------------------------------------- */

/* Extract an unsigned value from a byte array using bit-level addressing.
   start_bit: LSB position counting from bit 0 of data[0].
   length:    number of bits. Max 32. */
static uint32_t extract_bits(const uint8_t *data, int start_bit, int length) {
    uint32_t result = 0;
    for (int i = 0; i < length; i++) {
        int byte_idx = (start_bit + i) / 8;
        int bit_idx  = (start_bit + i) % 8;
        if ((data[byte_idx] >> bit_idx) & 1)
            result |= (1u << i);
    }
    return result;
}

static void decode_eec1(const uint8_t *data) {
    /* SPN 190 — Engine Speed: bytes 3-4, 0.125 RPM/bit, range 0-8031.875 */
    uint32_t raw_speed = extract_bits(data, 24, 16);
    float engine_rpm = raw_speed * 0.125f;

    /* SPN 512 — Driver's Demand Torque: byte 1, 1%/bit offset -125%, range -125 to 125% */
    uint8_t raw_torque_demand = data[1];
    int torque_demand = (int)raw_torque_demand - 125;

    /* SPN 513 — Actual Engine Torque: byte 2, same encoding */
    uint8_t raw_torque_actual = data[2];
    int torque_actual = (int)raw_torque_actual - 125;

    /* SPN 899 — Engine Torque Mode: bits 0-3 of byte 0 */
    uint8_t torque_mode = data[0] & 0x0F;

    printf("    [EEC1 SPNs]\n");
    printf("      SPN 190  Engine Speed          : %.2f RPM\n", engine_rpm);
    printf("      SPN 512  Demanded Torque       : %d %%\n", torque_demand);
    printf("      SPN 513  Actual Torque         : %d %%\n", torque_actual);
    printf("      SPN 899  Torque Mode           : %u\n", torque_mode);
}

static void decode_ccvs(const uint8_t *data) {
    /* SPN 84 — Wheel-Based Vehicle Speed: bytes 1-2, 1/256 km/h per bit */
    uint32_t raw_speed = extract_bits(data, 8, 16);
    float vehicle_kph = raw_speed / 256.0f;

    /* SPN 527 — Cruise Control Active: bit 6 of byte 0 */
    uint8_t cc_active = (data[0] >> 6) & 0x01;

    /* SPN 976 — Park Brake Switch: bit 2 of byte 3 */
    uint8_t park_brake = (data[3] >> 2) & 0x01;

    printf("    [CCVS SPNs]\n");
    printf("      SPN 84   Vehicle Speed         : %.2f km/h (%.2f mph)\n",
           vehicle_kph, vehicle_kph * 0.621371f);
    printf("      SPN 527  Cruise Control Active : %s\n", cc_active ? "Yes" : "No");
    printf("      SPN 976  Park Brake            : %s\n", park_brake ? "Engaged" : "Released");
}

static void decode_dm1(const uint8_t *data, int dlc) {
    /* DM1 reports active DTCs. Each DTC is 4 bytes.
       Byte 0: lamp status
       Bytes 1+: SPN (19 bits) | FMI (5 bits) | OC (7 bits) | CM (1 bit) per DTC */
    printf("    [DM1 — Active DTCs]\n");
    uint8_t lamp = data[0];
    printf("      Lamp status byte: 0x%02X\n", lamp);
    printf("        Protect lamp : %s\n", (lamp & 0x03) != 3 ? "ON" : "off");
    printf("        Amber lamp   : %s\n", ((lamp >> 2) & 0x03) != 3 ? "ON" : "off");
    printf("        Red lamp     : %s\n", ((lamp >> 4) & 0x03) != 3 ? "ON" : "off");

    int dtc_count = (dlc - 2) / 4;   /* byte 0 = lamps, byte 1 = reserved */
    if (dtc_count <= 0) {
        printf("      No active DTCs\n");
        return;
    }
    for (int i = 0; i < dtc_count; i++) {
        int base = 2 + i * 4;
        if (base + 3 >= dlc) break;
        uint32_t raw = ((uint32_t)data[base+3] << 24)
                     | ((uint32_t)data[base+2] << 16)
                     | ((uint32_t)data[base+1] <<  8)
                     |  (uint32_t)data[base+0];
        uint32_t spn = ((raw >> 5) & 0xFFFF) | (((raw >> 16) & 0x07) << 16);
        uint8_t  fmi = raw & 0x1F;
        uint8_t  oc  = (raw >> 24) & 0x7F;
        printf("      DTC %d: SPN %u  FMI %u  OC %u\n", i+1, spn, fmi, oc);
    }
}

static void decode_eet1(const uint8_t *data) {
    /* SPN 110 — Engine Coolant Temp: byte 0, 1°C/bit, offset -40°C */
    int coolant_c = (int)data[0] - 40;
    /* SPN 175 — Engine Oil Temp: bytes 2-3, 0.03125°C/bit, offset -273°C */
    uint32_t raw_oil = extract_bits(data, 16, 16);
    float oil_c = raw_oil * 0.03125f - 273.0f;

    printf("    [ET1 SPNs]\n");
    printf("      SPN 110  Coolant Temp          : %d °C (%d °F)\n",
           coolant_c, coolant_c * 9/5 + 32);
    printf("      SPN 175  Oil Temp              : %.1f °C (%.1f °F)\n",
           oil_c, oil_c * 9.0f/5.0f + 32.0f);
}

/* -------------------------------------------------------------------------
 * Top-level frame printer
 * ---------------------------------------------------------------------- */

void j1939_print_frame(const can_frame_t *frame) {
    if (!frame->extended) {
        printf("  [Standard 11-bit CAN — not a J1939 frame]\n");
        can_print_frame(frame);
        return;
    }

    j1939_header_t hdr;
    j1939_decode_header(frame->id, &hdr);

    printf("  Priority : %u\n",          hdr.priority);
    printf("  PGN      : 0x%05X (%u)\n", hdr.pgn, hdr.pgn);
    printf("  PGN Name : %s\n",           j1939_pgn_name(hdr.pgn));
    printf("  Src Addr : 0x%02X (%u)\n", hdr.src_addr, hdr.src_addr);
    if (hdr.pdu_format < 0xF0)
        printf("  Dst Addr : 0x%02X (%u)\n", hdr.pdu_specific, hdr.pdu_specific);
    printf("  DLC      : %d bytes\n",     frame->dlc);
    printf("  Data     :");
    for (int i = 0; i < frame->dlc; i++) printf(" %02X", frame->data[i]);
    printf("\n");

    /* SPN decoding for known PGNs */
    if (frame->dlc == 8) {
        switch (hdr.pgn) {
            case 0xF004: decode_eec1(frame->data);          break;
            case 0xFEF2: decode_ccvs(frame->data);          break;
            case 0xFECA: decode_dm1(frame->data, frame->dlc); break;
            case 0xFEEE: decode_eet1(frame->data);          break;
            default: break;
        }
    }
}
