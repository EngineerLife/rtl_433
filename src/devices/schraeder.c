/** @file
    Schrader TPMS protocol.

    Copyright (C) 2016 Benjamin Larsson
    and 2017 Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Schrader TPMS decoder.

Packet payload: 1 sync nibble and 8 bytes data, 17 nibbles:

    0 12 34 56 78 9A BC DE F0
    7 f6 70 3a 38 b2 00 49 49
    S PF FI II II II PP TT CC

- S: sync
- P: preamble (0xf)
- F: flags
- I: id (28 bit)
- P: pressure from 0 bar to 6.375 bar, resolution of 25 mbar/hectopascal per bit
- T: temperature from -50 C to 205 C (1 bit = 1 temperature count 1 C)
- C: CRC8 from nibble 1 to E
*/

#include <stdbool.h>
#include "decoder.h"

static int schraeder_callback(r_device *decoder, bitbuffer_t *bitbuffer) {
    data_t *data;
    uint8_t b[8];
    int serial_id;
    char id_str[9];
    int flags;
    char flags_str[3];
    int pressure;    // mbar/hectopascal
    int temperature; // deg C

    /* Reject wrong amount of bits */
    if (bitbuffer->bits_per_row[0] != 68)
        return DECODE_ABORT_LENGTH;

    /* Shift the buffer 4 bits to remove the sync bits */
    bitbuffer_extract_bytes(bitbuffer, 0, 4, b, 64);

    /* Calculate the crc */
    if (b[7] != crc8(b, 7, 0x07, 0xf0)) {
        return DECODE_FAIL_MIC;
    }

    /* Get data */
    serial_id   = (b[1] & 0x0F) << 24 | b[2] << 16 | b[3] << 8 | b[4];
    flags       = (b[0] & 0x0F) << 4 | b[1] >> 4;
    pressure    = b[5] * 25;
    temperature = b[6] - 50;
    sprintf(id_str, "%07X", serial_id);
    sprintf(flags_str, "%02x", flags);

    data = data_make(
            "model",            "",             DATA_STRING, "Schrader",
            "type",             "",             DATA_STRING, "TPMS",
            "flags",            "",             DATA_STRING, flags_str,
            "id",               "ID",           DATA_STRING, id_str,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.1f kPa", DATA_DOUBLE, (double)pressure*0.1,
            "temperature_C",    "Temperature",  DATA_FORMAT, "%.0f C", DATA_DOUBLE, (double)temperature,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

/**
TPMS Model: Schrader Electronics EG53MA4.
Contributed by: Leonardo Hamada (hkazu).

Also Schrader PA66-GF35 (OPEL OEM 13348393) TPMS Sensor.

Probable packet payload:

    SSSSSSSSSS ???????? IIIIII TT PP CC

- S: sync
- ?: might contain the preamble, status and battery flags
- I: id (24 bits), could extend into flag bits (?)
- P: pressure, 25 mbar per bit
- T: temperature, degrees Fahrenheit
- C: checksum, sum of byte data modulo 256
*/
static int schrader_EG53MA4_callback(r_device *decoder, bitbuffer_t *bitbuffer) {
    data_t *data;
    uint8_t b[10];
    int serial_id;
    char id_str[9];
    unsigned flags;
    char flags_str[9];
    int pressure;    // mbar
    int temperature; // degree Fahrenheit
    int checksum;

    /* Check for incorrect number of bits received */
    if (bitbuffer->bits_per_row[0] != 120)
        return DECODE_ABORT_LENGTH;

    /* Discard the first 40 bits */
    bitbuffer_extract_bytes(bitbuffer, 0, 40, b, 80);

    /* Calculate the checksum */
    checksum = add_bytes(b, 9) & 0xff;
    if (checksum != b[9]) {
        return DECODE_FAIL_MIC;
    }

    /* Get data */
    serial_id   = (b[4] << 16) | (b[5] << 8) | b[6];
    flags       = ((unsigned)b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
    pressure    = b[7] * 25;
    temperature = b[8];
    sprintf(id_str, "%06X", serial_id);
    sprintf(flags_str, "%08x", flags);

    data = data_make(
            "model",            "",             DATA_STRING, "Schrader-EG53MA4",
            "type",             "",             DATA_STRING, "TPMS",
            "flags",            "",             DATA_STRING, flags_str,
            "id",               "ID",           DATA_STRING, id_str,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.1f kPa", DATA_DOUBLE, (double)pressure*0.1,
            "temperature_F",    "Temperature",  DATA_FORMAT, "%.1f F", DATA_DOUBLE, (double)temperature,
            "mic",              "Integrity",    DATA_STRING, "CHECKSUM",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}


static uint8_t find_byte_parity(const uint8_t byte) {
    uint8_t parity = 0x01;
    parity ^= (byte>>7)&0x1;
    parity ^= (byte>>6)&0x1;
    parity ^= (byte>>5)&0x1;
    parity ^= (byte>>4)&0x1;
    parity ^= (byte>>3)&0x1;
    parity ^= (byte>>2)&0x1;
    parity ^= byte&0x1;
    return (parity&0x1);
}

static bool parity_check(const uint8_t* data,const uint8_t LENGTH) {
    uint8_t parity;
    for(int j = 0;j<LENGTH;++j) {
        parity ^= find_byte_parity(data[j]);
    }
    return (parity&0x1);
}

static int schrader_SE3_callback(r_device *decoder, bitbuffer_t *bitbuffer) {
    data_t *data;
    char id_str[9];
    uint8_t flags;
    char flags_str[9];
    uint8_t b[5];

    /* Check for incorrect number of bits received */
    if (bitbuffer->bits_per_row[0] != 52 && bitbuffer->bits_per_row[0] != 53)
        return DECODE_ABORT_LENGTH;

    /* Discard the first 16 bits */
    bitbuffer_extract_bytes(bitbuffer, 0, 16, b, 40);
    /* Calculate the checksum */
    /*checksum = add_bytes(b, 9) & 0xff;
    if (checksum != b[9]) {
        return DECODE_FAIL_MIC;
    }*/
    //TODO implement parity check

    /* Get data */
    // bits 1-3
    uint8_t PARITY_VAL = (b[4]&0x10)>>4;
    b[4]=b[4]&0xE0;
    b[5]=0x00;
    if(parity_check(b,5) != PARITY_VAL)
    {
        //return DECODE_FAIL_MIC;
    }
    flags = (b[0] & 0xE0) >> 5; //first three bits
    // bits 4-27
    const uint8_t ID_BYTE_1 = (((b[0] & 0x1F) << 3) | ((b[1]&0xE0)>>5));
    const uint8_t ID_BYTE_2 = ((b[1]&0x1F) << 3) | ((b[2]&0xE0)>>5);
    const uint8_t ID_BYTE_3 = ((b[2]&0x1F) << 3) | ((b[3]&0xE0)>>5);

    const SERIAL_ID   = ID_BYTE_1<< 16|
                        ID_BYTE_2<< 8 |
                        ID_BYTE_3;

    const uint8_t PRESSURE_INT_VAL    = ((b[3]&0x1F) << 3) | ((b[4]&0xE0)>>5);
    sprintf(id_str, "%06X", SERIAL_ID);
    sprintf(flags_str, "%02x", flags);

    data = data_make(
            "model",            "",             DATA_STRING, "Schrader-SE3",
            "type",             "",             DATA_STRING, "TPMS",
            "flags",            "",             DATA_STRING, flags_str,
            "id",               "ID",           DATA_STRING, id_str,
            "pressure_kPa",     "Pressure",     DATA_FORMAT, "%.1f PSI", DATA_DOUBLE, (double)PRESSURE_INT_VAL*0.2 + 0.1,
            "mic",              "Integrity",    DATA_STRING, "N/A",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static char *output_fields[] = {
        "model",
        "type",
        "id",
        "flags",
        "pressure_kPa",
        "temperature_C",
        "mic",
        NULL,
};

static char *output_fields_EG53MA4[] = {
        "model",
        "type",
        "id",
        "flags",
        "pressure_kPa",
        "temperature_F",
        "mic",
        NULL,
};

static char *output_fields_SE3[] = {
        "model",
        "type",
        "flags",
        "id",
        "pressure_kPa",
        "mic",
        NULL,
};

r_device schraeder = {
        .name        = "Schrader TPMS",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 120,
        .long_width  = 0,
        .reset_limit = 480,
        .decode_fn   = &schraeder_callback,
        .disabled    = 0,
        .fields      = output_fields,
};

r_device schrader_EG53MA4 = {
        .name        = "Schrader TPMS EG53MA4, PA66GF35",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 123,
        .long_width  = 0,
        .reset_limit = 300,
        .decode_fn   = &schrader_EG53MA4_callback,
        .disabled    = 0,
        .fields      = output_fields_EG53MA4,
};

r_device schrader_SE3 = {
        .name        = "Schrader SE3",
        .modulation  = OOK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 120,
        .long_width  = 0,
        .reset_limit = 300,
        .decode_fn   = &schrader_SE3_callback,
        .disabled    = 0,
        .fields      = output_fields_SE3,

};