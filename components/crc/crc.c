#include "crc.h"

/**
 * This function is part of Cleanflight & iNAV.
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
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

/**
 * Does the same as crc8_dvb_s2 just way more efficient (CPU) but with a little bit more RAM usage
 * @param crc The current crc value
 * @param a The next byte
 * @return The new crc value
 */
uint8_t crc8_dvb_s2_table(uint8_t crc, unsigned char a)
{
    return (uint8_t) (crc_dvb_s2_table[(crc ^ a)] & 0xff);
}

/**
 * @brief Calculate CRC32 of a string
 * 
 * @param crc Initial CRC value (0 if not known)
 * @param buf Buffer containing string
 * @param len Length of the string
 * @return uint32_t CRC32 value
 */
uint32_t calc_crc32(uint32_t crc, unsigned char *buf, size_t len) {
    int k;
    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
    }
    return ~crc;
}

/**
 * @brief Check CRC32 of a string
 * 
 * @param buf Buffer containing string
 * @param msg_length Length of the string
 * @return 1 if CRC is good else 0
 */
int crc_ok(uint8_t *buf, int msg_length) {
    uint32_t c_crc = calc_crc32((uint32_t) 0, buf, (size_t) (msg_length - 4));
    uint8_t crc_bytes[4];
    crc_bytes[0] = c_crc;
    crc_bytes[1] = c_crc >> 8;
    crc_bytes[2] = c_crc >> 16;
    crc_bytes[3] = c_crc >> 24;
    if ((crc_bytes[0] == buf[msg_length - 4]) && (crc_bytes[1] == buf[msg_length - 3]) &&
        (crc_bytes[2] == buf[msg_length - 2]) && (crc_bytes[3] == buf[msg_length - 1])) {
        return 1;
    } else {
        return 0;
    }
}
