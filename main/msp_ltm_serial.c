/*
 *   This file is part of DroneBridge: https://github.com/DroneBridge/ESP32
 *
 *   Copyright 2018 Wolfgang Christl
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "msp_ltm_serial.h"
#include "db_crc.h"

/**
 * This function is part of Cleanflight/iNAV.
 *
 * Optimized for crc performance in the DroneBridge project and ">" & "<" added
 * for bidirectional parsing. Included logic to detect LTM telemetry.
 * Added by Wolfgang Christl
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
bool parse_msp_ltm_byte(msp_ltm_port_t *msp_ltm_port, uint8_t new_byte) {
    switch (msp_ltm_port->parse_state) {
        default:
        case IDLE:
            if (new_byte == '$') {
                msp_ltm_port->mspVersion = MSP_V1;
                msp_ltm_port->parse_state = HEADER_START;
                msp_ltm_port->ltm_frame_buffer[0] = '$';
            } else { return false; }
            msp_ltm_port->ltm_payload_cnt = 0;
            msp_ltm_port->checksum1 = 0;
            break;

        case HEADER_START:
            switch (new_byte) {
                case 'M':
                    msp_ltm_port->parse_state = MSP_HEADER_M;
                    break;
                case 'X':
                    msp_ltm_port->parse_state = MSP_HEADER_X;
                    break;
                case 'T':
                    msp_ltm_port->parse_state = LTM_HEADER;
                    msp_ltm_port->ltm_frame_buffer[1] = 'T';
                    break;
                default:
                    msp_ltm_port->parse_state = IDLE;
                    return false;
            }
            break;

        case LTM_HEADER:
            switch (new_byte) {
                case 'A':
                    msp_ltm_port->ltm_type = LTM_TYPE_A;
                    msp_ltm_port->parse_state = LTM_TYPE_IDENT;
                    break;
                case 'G':
                    msp_ltm_port->ltm_type = LTM_TYPE_G;
                    msp_ltm_port->parse_state = LTM_TYPE_IDENT;
                    break;
                case 'N':
                    msp_ltm_port->ltm_type = LTM_TYPE_N;
                    msp_ltm_port->parse_state = LTM_TYPE_IDENT;
                    break;
                case 'O':
                    msp_ltm_port->ltm_type = LTM_TYPE_O;
                    msp_ltm_port->parse_state = LTM_TYPE_IDENT;
                    break;
                case 'S':
                    msp_ltm_port->ltm_type = LTM_TYPE_S;
                    msp_ltm_port->parse_state = LTM_TYPE_IDENT;
                    break;
                case 'X':
                    msp_ltm_port->ltm_type = LTM_TYPE_X;
                    msp_ltm_port->parse_state = LTM_TYPE_IDENT;
                    break;
                default:
                    msp_ltm_port->parse_state = IDLE;
                    break;
            }
            msp_ltm_port->ltm_frame_buffer[2] = new_byte;
            msp_ltm_port->ltm_payload_cnt = 0;
            break;

        case LTM_TYPE_IDENT:
            msp_ltm_port->ltm_payload_cnt++;
            msp_ltm_port->ltm_frame_buffer[2 + msp_ltm_port->ltm_payload_cnt] = new_byte;
            msp_ltm_port->checksum1 ^= new_byte;
            switch (msp_ltm_port->ltm_type) {
                case LTM_TYPE_A:
                case LTM_TYPE_N:
                case LTM_TYPE_X:
                    if (msp_ltm_port->ltm_payload_cnt == LTM_TYPE_A_PAYLOAD_SIZE) msp_ltm_port->parse_state = LTM_CRC;
                    break;
                case LTM_TYPE_G:
                case LTM_TYPE_O:
                    if (msp_ltm_port->ltm_payload_cnt == LTM_TYPE_G_PAYLOAD_SIZE) msp_ltm_port->parse_state = LTM_CRC;
                    break;
                case LTM_TYPE_S:
                    if (msp_ltm_port->ltm_payload_cnt == LTM_TYPE_S_PAYLOAD_SIZE) msp_ltm_port->parse_state = LTM_CRC;
                    break;
                default:
                    msp_ltm_port->parse_state = IDLE;
                    return false;
            }
            break;

        case LTM_CRC:
            msp_ltm_port->ltm_frame_buffer[3 + msp_ltm_port->ltm_payload_cnt] = new_byte;
            if (msp_ltm_port->checksum1 == new_byte) {
                msp_ltm_port->parse_state = LTM_PACKET_RECEIVED;
            } else {
                msp_ltm_port->parse_state = IDLE;
            }
            break;

        case MSP_HEADER_M:
            if (new_byte == '>' || new_byte == '<' || new_byte == '!') {
                msp_ltm_port->offset = 0;
                msp_ltm_port->checksum1 = 0;
                msp_ltm_port->checksum2 = 0;
                msp_ltm_port->parse_state = MSP_HEADER_V1;
            } else {
                msp_ltm_port->parse_state = IDLE;
            }
            break;

        case MSP_HEADER_X:
            if (new_byte == '>' || new_byte == '<' || new_byte == '!') {
                msp_ltm_port->offset = 0;
                msp_ltm_port->checksum2 = 0;
                msp_ltm_port->mspVersion = MSP_V2_NATIVE;
                msp_ltm_port->parse_state = MSP_HEADER_V2_NATIVE;
            } else {
                msp_ltm_port->parse_state = IDLE;
            }
            break;

        case MSP_HEADER_V1:
            msp_ltm_port->inBuf[msp_ltm_port->offset++] = new_byte;
            msp_ltm_port->checksum1 ^= new_byte;
            if (msp_ltm_port->offset == sizeof(mspHeaderV1_t)) {
                mspHeaderV1_t *hdr = (mspHeaderV1_t *) &msp_ltm_port->inBuf[0];
                // No need to check incoming buffer size limit for V1
                // because the buffer is 512 which is bigger than the INTMAX for byte
                if (hdr->cmd == MSP_V2_FRAME_ID) {
                    if (hdr->size >= sizeof(mspHeaderV2_t) + 1) {
                        msp_ltm_port->mspVersion = MSP_V2_OVER_V1;
                        msp_ltm_port->parse_state = MSP_HEADER_V2_OVER_V1;
                    } else {
                        msp_ltm_port->parse_state = IDLE;
                        return false;
                    }
                } else {
                    msp_ltm_port->dataSize = hdr->size;
                    msp_ltm_port->cmdMSP = hdr->cmd;
                    msp_ltm_port->cmdFlags = 0;
                    msp_ltm_port->offset = 0;
                    msp_ltm_port->parse_state = msp_ltm_port->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;
                }
            }
            break;

        case MSP_PAYLOAD_V1:
            msp_ltm_port->inBuf[msp_ltm_port->offset++] = new_byte;
            msp_ltm_port->checksum1 ^= new_byte;
            if (msp_ltm_port->offset == msp_ltm_port->dataSize) {
                msp_ltm_port->parse_state = MSP_CHECKSUM_V1;
            }
            break;

        case MSP_CHECKSUM_V1:
            if (msp_ltm_port->checksum1 == new_byte) {
                msp_ltm_port->parse_state = MSP_PACKET_RECEIVED;
            } else {
                msp_ltm_port->parse_state = IDLE;
            }
            break;

        case MSP_HEADER_V2_OVER_V1:
            msp_ltm_port->inBuf[msp_ltm_port->offset++] = new_byte;
            msp_ltm_port->checksum1 ^= new_byte;
            msp_ltm_port->checksum2 = crc8_dvb_s2_table(msp_ltm_port->checksum2, new_byte);
            if (msp_ltm_port->offset == (sizeof(mspHeaderV2_t) + sizeof(mspHeaderV1_t))) {
                mspHeaderV2_t *hdrv2 = (mspHeaderV2_t *) &msp_ltm_port->inBuf[sizeof(mspHeaderV1_t)];
                msp_ltm_port->dataSize = hdrv2->size;
                if (hdrv2->size > MSP_PORT_INBUF_SIZE) {
                    msp_ltm_port->parse_state = IDLE;
                    return false;
                } else {
                    msp_ltm_port->cmdMSP = hdrv2->cmd;
                    msp_ltm_port->cmdFlags = hdrv2->flags;
                    msp_ltm_port->offset = 0;
                    msp_ltm_port->parse_state =
                            msp_ltm_port->dataSize > 0 ? MSP_PAYLOAD_V2_OVER_V1 : MSP_CHECKSUM_V2_OVER_V1;
                }
            }
            break;

        case MSP_PAYLOAD_V2_OVER_V1:
            msp_ltm_port->checksum2 = crc8_dvb_s2_table(msp_ltm_port->checksum2, new_byte);
            msp_ltm_port->checksum1 ^= new_byte;
            msp_ltm_port->inBuf[msp_ltm_port->offset++] = new_byte;

            if (msp_ltm_port->offset == msp_ltm_port->dataSize) {
                msp_ltm_port->parse_state = MSP_CHECKSUM_V2_OVER_V1;
            }
            break;

        case MSP_CHECKSUM_V2_OVER_V1:
            msp_ltm_port->checksum1 ^= new_byte;
            if (msp_ltm_port->checksum2 == new_byte) {
                msp_ltm_port->parse_state = MSP_CHECKSUM_V1;
            } else {
                msp_ltm_port->parse_state = IDLE;
                return false;
            }
            break;

        case MSP_HEADER_V2_NATIVE:
            msp_ltm_port->inBuf[msp_ltm_port->offset++] = new_byte;
            msp_ltm_port->checksum2 = crc8_dvb_s2_table(msp_ltm_port->checksum2, new_byte);
            if (msp_ltm_port->offset == sizeof(mspHeaderV2_t)) {
                mspHeaderV2_t *hdrv2 = (mspHeaderV2_t *) &msp_ltm_port->inBuf[0];
                if (hdrv2->size > MSP_PORT_INBUF_SIZE) {
                    msp_ltm_port->parse_state = IDLE;
                } else {
                    msp_ltm_port->dataSize = hdrv2->size;
                    msp_ltm_port->cmdMSP = hdrv2->cmd;
                    msp_ltm_port->cmdFlags = hdrv2->flags;
                    msp_ltm_port->offset = 0;
                    msp_ltm_port->parse_state =
                            msp_ltm_port->dataSize > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
                }
            }
            break;

        case MSP_PAYLOAD_V2_NATIVE:
            msp_ltm_port->checksum2 = crc8_dvb_s2_table(msp_ltm_port->checksum2, new_byte);
            msp_ltm_port->inBuf[msp_ltm_port->offset++] = new_byte;

            if (msp_ltm_port->offset == msp_ltm_port->dataSize) {
                msp_ltm_port->parse_state = MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_CHECKSUM_V2_NATIVE:
            if (msp_ltm_port->checksum2 == new_byte) {
                msp_ltm_port->parse_state = MSP_PACKET_RECEIVED;
            } else {
                msp_ltm_port->parse_state = IDLE;
                return false;
            }
            break;
    }
    return true;
}
