#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "main.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(sensirion_sht4x)
#error "No sensirion,sht4x compatible node found in the device tree"
#endif

static const struct device* sht40_device;

#if CONFIG_BT_ID_MAX > 1
#error "This application supports only one Bluetooth identity"
#endif

static const uint8_t* const uicr_predefined_key = (const uint8_t*)(0x10001000 + 0x080); // UICR COUSTOM[15:0]

static uint8_t       encryption_nonce[16] = { 0 }; // actually 13 bytes
static const uint8_t predefined_key[16]   = { 0x8b, 0xa5, 0x91, 0xa5, 0xef, 0x8f, 0xd5, 0x99,
                                              0x90, 0x31, 0x6d, 0x38, 0xe0, 0x4a, 0xe9, 0xed };

#define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY, BT_ADV_MIN_INTERVAL, BT_ADV_MAX_INTERVAL, NULL)

static struct bthome_raw_data raw_data = { .battery_object_id     = BTHOMEV2_OBJID_BATTERY_U8_1,
                                           .battery_level         = 42,
                                           .temperature_object_id = BTHOMEV2_OBJID_TEMPERATURE_S16_0P01,
                                           .temperature           = 0000,
                                           .humidity_object_id    = BTHOMEV2_OBJID_HUMIDITY_U16_0P01,
                                           .humidity              = 0000 };

static struct bthome_payload advertising_payload = { .service_uuid     = { BT_UUID_16_ENCODE(BTHOMEV2_SERVICE_UUID) },
                                                     .device_info_byte = BTHOMEV2_DIB_ENCRYPTED | BTHOMEV2_DIB_VER,
                                                     .encrypted_data   = { 0 },
                                                     .counter          = 0x00000000,
                                                     .mic              = { 0 } };

static struct bt_data advertising_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_SVC_DATA16, &advertising_payload, sizeof(advertising_payload))
};

static struct bt_data scan_response_data[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

#include <nrfx_saadc.h>
#define SAADC_INPUT_PIN NRFX_ANALOG_INTERNAL_VDD

static int16_t              battery_sample_voltage;
static nrfx_saadc_channel_t battery_sample_channel  = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);
static int                  battery_sample_time_due = 0;

void battery_sample()
{
    LOG_INF("fetching saadc");
    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != 0)
    {
        LOG_ERR("nrfx_saadc_mode_trigger error: %d", (int)err);
        return;
    }
    battery_sample_channel.channel_config.gain = NRF_SAADC_GAIN1_6;

    err = nrfx_saadc_channels_config(&battery_sample_channel, 1);
    if (err != 0)
    {
        LOG_ERR("nrfx_saadc_channels_config error: %d", (int)err);
        return;
    }
    err = nrfx_saadc_simple_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_128X, NULL);
    if (err != 0)
    {
        LOG_ERR("nrfx_saadc_simple_mode_set error: %d", (int)err);
        return;
    }
    err = nrfx_saadc_buffer_set(&battery_sample_voltage, 1);
    if (err != 0)
    {
        LOG_ERR("nrfx_saadc_buffer_set error: %d", (int)err);
        return;
    }
    err = nrfx_saadc_offset_calibrate(NULL);
    if (err != 0)
    {
        LOG_ERR("nrfx_saadc_offset_calibrate error: %d", (int)err);
        return;
    }
    err = nrfx_saadc_mode_trigger();
    if (err != 0)
    {
        LOG_ERR("nrfx_saadc_mode_trigger error: %d", (int)err);
        return;
    }
    int batt_volt          = ((600 * 6) * battery_sample_voltage) / ((1 << 12));
    raw_data.battery_level = (batt_volt > 3200) ? 100 : (batt_volt < 2500) ? 0 : (batt_volt - 2500) / 7;
    nrfx_saadc_uninit(); // save power
    LOG_INF(
        "sampled battery voltage: %d mV, orignal data: %hd, level: %hhu%%",
        batt_volt,
        battery_sample_voltage,
        raw_data.battery_level);
}
void battery_sample_timer_handler(struct k_timer* timer)
{
    battery_sample_time_due = 1;
}

K_TIMER_DEFINE(battery_sample_timer, battery_sample_timer_handler, NULL);
static int sensor_sample_time_due = 0;

void sensor_sample()
{
    if (sht40_device == NULL)
    {
        LOG_ERR("SHT4X device pointer is NULL");
        return;
    }

    if (!device_is_ready(sht40_device))
    {
        LOG_ERR("sht40 %s is not ready", sht40_device->name);
        return;
    }
    else
    {
        LOG_INF("sht40 %s device ready", sht40_device->name);
    }

    if (sensor_sample_fetch(sht40_device))
    {
        LOG_ERR("fetch sample from SHT4X device failed ");
        return;
    }

    struct sensor_value sht40_t  = { 0 };
    struct sensor_value sht40_rh = { 0 };

    sensor_channel_get(sht40_device, SENSOR_CHAN_AMBIENT_TEMP, &sht40_t); // data zero initialized so failure is
    sensor_channel_get(sht40_device, SENSOR_CHAN_HUMIDITY, &sht40_rh);

    raw_data.temperature = (int16_t)(sht40_t.val1 * 100 + sht40_t.val2 / 10000);
    raw_data.humidity    = (uint16_t)(sht40_rh.val1 * 100 + sht40_rh.val2 / 10000);

    LOG_INF("sample fetched from SHT4X device: temp=%hd, humidity=%hu", raw_data.temperature, raw_data.humidity);
}
void sensor_sample_timer_handler(struct k_timer* timer)
{
    sensor_sample_time_due = 1;
}
K_TIMER_DEFINE(sensor_sample_timer, sensor_sample_timer_handler, NULL);

static int encrypt_init()
{
    int result = bt_rand(&(advertising_payload.counter), sizeof(advertising_payload.counter));
    if (result)
    {
        LOG_INF("counter initlization by bt_rand() filling failed: %d", result);
        return -1;
    }

    bt_addr_le_t mac_addr[CONFIG_BT_ID_MAX];
    size_t       mac_count = CONFIG_BT_ID_MAX;
    bt_id_get(mac_addr, &mac_count);
    if (mac_count == 0)
    {
        LOG_INF("nonce initlization by bt_id_get() failed: no MAC addresses found");
        return -1;
    }
    for (size_t i = 0; i < mac_count; i++)
    {
        char addr_str[BT_ADDR_LE_STR_LEN] = { 0 };
        bt_addr_le_to_str(&mac_addr[i], addr_str, sizeof(addr_str));
        LOG_INF("device MAC address %u: %s", (unsigned int)i, addr_str);
    }

    encryption_nonce[5] = mac_addr[0].a.val[0];
    encryption_nonce[4] = mac_addr[0].a.val[1];
    encryption_nonce[3] = mac_addr[0].a.val[2];
    encryption_nonce[2] = mac_addr[0].a.val[3];
    encryption_nonce[1] = mac_addr[0].a.val[4];
    encryption_nonce[0] = mac_addr[0].a.val[5];

    memcpy(encryption_nonce + 6, advertising_payload.service_uuid, 2);
    memcpy(encryption_nonce + 8, &(advertising_payload.device_info_byte), 1);
    memcpy(encryption_nonce + 9, &(advertising_payload.counter), 4);
    LOG_INF("encryption initialization completed");

    return 0;
}

static int encrypt_payload(
    struct bthome_payload*        payload,
    const struct bthome_raw_data* raw_data,
    uint8_t*                      nonce,
    const uint8_t*                key)
{
    if (payload == NULL || raw_data == NULL || nonce == NULL || key == NULL)
    {
        LOG_INF("encrypt_payload() invalid parameter");
        return -1;
    }

    memcpy(nonce + 9, &(payload->counter), 4);
    uint8_t enc_data_buffer[sizeof(struct bthome_raw_data) + 4] = { 0 };

    int result =
        bt_ccm_encrypt(key, nonce, (uint8_t*)raw_data, sizeof(struct bthome_raw_data), NULL, 0, enc_data_buffer, 4);
    if (result)
    {
        LOG_INF("bt_ccm_encrypt() failed: %d", result);
        return result;
    }

    memcpy(&(payload->encrypted_data), enc_data_buffer, sizeof(struct bthome_raw_data));
    memcpy(&(payload->mic), enc_data_buffer + sizeof(struct bthome_raw_data), 4);

    return result;
}

static void bt_ready(int result)
{
    if (result)
    {
        LOG_ERR("bluetooth init failed: %d", result);
        return;
    }

    result = bt_le_adv_start(
        ADV_PARAM, advertising_data, ARRAY_SIZE(advertising_data), scan_response_data, ARRAY_SIZE(scan_response_data));
    if (result)
    {
        LOG_ERR(
            "bluetooth adsr failed to set data: %d, sizeof(advertising_payload):%d",
            result,
            sizeof(advertising_payload));
        return;
    }
    LOG_INF("bluetooth advertising started");
}

int main(void)
{
    LOG_INF("Alan nRF52832SHT40 BTHomev2 Sensor");

    LOG_INF("fetching sht40");
    sht40_device = DEVICE_DT_GET_ANY(sensirion_sht4x);
    if (sht40_device == NULL)
    {
        LOG_ERR("sht40 device not found");
        return -1;
    }
    else
    {
        LOG_INF("sht40 device found: %s", sht40_device->name);
    }

    LOG_INF("starting battery monitoring timer");
    k_timer_start(&battery_sample_timer, K_NO_WAIT, K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));

    LOG_INF("starting sensor timer");
    k_timer_start(&sensor_sample_timer, K_NO_WAIT, K_MSEC(SENSOR_SAMPLE_INTERVAL_MS));

    LOG_INF("enabling bluetooth");
    int result = bt_enable(bt_ready);
    if (result)
    {
        LOG_ERR("bluetooth init failed: %d", result);
        return -1;
    }
    else
    {
        LOG_INF("bluetooth ready");
    }
    result = encrypt_init();
    if (result)
    {
        LOG_ERR("encryption initialization failed: %d", result);
        return -1;
    }
    if (memcmp(uicr_predefined_key, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16) != 0)
    {
        LOG_INF("using UICR predefined key for encryption");
    }
    else
    {
        LOG_INF("using default predefined key for encryption");
    }
    while (1)
    {
        if (battery_sample_time_due)
        {
            battery_sample_time_due = 0;
            battery_sample();
        }
        if (sensor_sample_time_due)
        {
            sensor_sample_time_due = 0;
            sensor_sample();
        }
        if (memcmp(uicr_predefined_key, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16) != 0)
        {
            result = encrypt_payload(&advertising_payload, &raw_data, encryption_nonce, uicr_predefined_key);
        }
        else
        {
            result = encrypt_payload(&advertising_payload, &raw_data, encryption_nonce, predefined_key);
        }

        if (result)
        {
            LOG_ERR("payload encryption failed: %d", result);
            continue;
        }
        result = bt_le_adv_update_data(
            advertising_data, ARRAY_SIZE(advertising_data), scan_response_data, ARRAY_SIZE(scan_response_data));
        if (result)
        {
            LOG_ERR("bluetooth adsr update failed to set data: %d", result);
            continue;
        }
        advertising_payload.counter++;

        k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL)));
    }
    return 0;
}