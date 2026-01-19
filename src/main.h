#pragma once

#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>

#define BTHOMEV2_SERVICE_UUID               0xFCD2

#define BTHOMEV2_DIB_ENCRYPTED              (0x01 << 0)
#define BTHOMEV2_DIB_RESERVED1              (0x01 << 1)
#define BTHOMEV2_DIB_TRIGGER_BASED          (0x01 << 2)
#define BTHOMEV2_DIB_RESERVED3              (0x01 << 3)
#define BTHOMEV2_DIB_RESERVED4              (0x01 << 4)
#define BTHOMEV2_DIB_VER                    (0x02 << 5)

#define BTHOMEV2_OBJID_BATTERY_U8_1         0x01
#define BTHOMEV2_OBJID_TEMPERATURE_S16_0P01 0x02
#define BTHOMEV2_OBJID_HUMIDITY_U16_0P01    0x03

struct bthome_raw_data
{
    uint8_t  battery_object_id;
    uint8_t  battery_level;
    uint8_t  temperature_object_id;
    int16_t  temperature;
    uint8_t  humidity_object_id;
    uint16_t humidity;
} __attribute__((packed));

struct bthome_payload
{
    uint8_t                service_uuid[2]; // little-endian
    uint8_t                device_info_byte;
    struct bthome_raw_data encrypted_data;
    uint32_t               counter; // little-endian
    uint8_t                mic[4];
} __attribute__((packed));

#define BT_ADV_MIN_INTERVAL         8224  // 5.140s
#define BT_SHORTEN_ADV_MIN_INTERVAL 4112  // 2.570s
#define BT_ADV_MAX_INTERVAL         10280 // 6.425s
#define BT_SHORTEN_ADV_MAX_INTERVAL 6168  // 3.855s

#define BATTERY_SAMPLE_INTERVAL_MS  600000 // 600s
#define SENSOR_SAMPLE_INTERVAL_MS   20000  // 20s

#define BT_TICK_TO_MSEC(ticks)      ((ticks) * 10 / 16)

#endif // _MAIN_H_