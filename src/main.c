#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "main.h"

#if !DT_HAS_COMPAT_STATUS_OKAY(sensirion_sht4x)
#error "No sensirion,sht4x compatible node found in the device tree"
#endif

static const struct device* sht40_device;

static int16_t  historical_temperature = 0;
static uint16_t historical_humidity    = 0;
static int16_t  perv_temperature       = 0;
static uint16_t perv_humidity          = 0;

#if CONFIG_BT_ID_MAX > 1
#error "This application supports only one Bluetooth identity"
#endif
static uint8_t encryption_nonce[16] = { 0 }; // actually 13 bytes
// static const uint8_t* const predefined_key       = 0x10001000 + 0x080; // UICR COUSTOM[15:0]
static const uint8_t predefined_key[16] = { 0x8b, 0xa5, 0x91, 0xa5, 0xef, 0x8f, 0xd5, 0x99,
                                            0x90, 0x31, 0x6d, 0x38, 0xe0, 0x4a, 0xe9, 0xed };

// #define ADV_PARAM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_EXT_ADV, BT_GAP_ADV_SLOW_INT_MIN,
// BT_GAP_ADV_SLOW_INT_MAX, NULL)
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
#define SAADC_INPUT_PIN            NRFX_ANALOG_INTERNAL_VDD
#define BATTERY_SAMPLE_INTERVAL_MS 300000

static int16_t              battery_sample_voltage;
static nrfx_saadc_channel_t battery_sample_channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);

void battery_sample_timer_handler(struct k_timer* timer)
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
    err = nrfx_saadc_simple_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_DISABLED, NULL);
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
    nrfx_saadc_uninit();
    LOG_INF(
        "sampled battery voltage: %d mV, orignal data: %hd, level: %hhu%%",
        batt_volt,
        battery_sample_voltage,
        raw_data.battery_level);
}
K_TIMER_DEFINE(battery_sample_timer, battery_sample_timer_handler, NULL);

//
//
// Sensor
//
//

K_THREAD_STACK_DEFINE(sensor_thread_stack, 1024);
struct k_thread sensor_thread;

void sensor_handle(void* p1, void* p2, void* p3)
{
    LOG_INF("starting sensor thread");
    k_timer_start(&battery_sample_timer, K_NO_WAIT, K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));

    LOG_INF("fetching sht40");
    sht40_device = DEVICE_DT_GET_ANY(sensirion_sht4x);
    if (sht40_device == NULL)
    {
        LOG_ERR("sht40 device not found");
        return;
    }
    else
    {
        LOG_INF("sht40 device found: %s", sht40_device->name);
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

    while (1)
    {
        //
        // fetch temperature and humidity from SHT4X sensor
        //
        if (sht40_device == NULL)
        {
            LOG_ERR("SHT4X device pointer is NULL");
            continue;
        }
        if (sensor_sample_fetch(sht40_device))
        {
            LOG_ERR("fetch sample from SHT4X device failed ");
            continue;
        }

        struct sensor_value sht40_t  = { 0 };
        struct sensor_value sht40_rh = { 0 };

        sensor_channel_get(sht40_device, SENSOR_CHAN_AMBIENT_TEMP, &sht40_t); // data zero initialized so failure is
        sensor_channel_get(sht40_device, SENSOR_CHAN_HUMIDITY, &sht40_rh);

        perv_temperature     = raw_data.temperature;
        perv_humidity        = raw_data.humidity;
        raw_data.temperature = (int16_t)(sht40_t.val1 * 100 + sht40_t.val2 / 10000);
        raw_data.humidity    = (uint16_t)(sht40_rh.val1 * 100 + sht40_rh.val2 / 10000);

        LOG_INF("sample fetched from SHT4X device: temp=%hd, humidity=%hu", raw_data.temperature, raw_data.humidity);

        k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL)));
    }
}

//
//
// BT
//
//

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
    // LOG_HEXDUMP_INF(raw_data, sizeof(struct bthome_raw_data), "raw_data to encrypt:");
    // LOG_HEXDUMP_INF(&(payload->counter), 4, "counter to encrypt:");
    // LOG_HEXDUMP_INF(nonce, 13, "nonce to encrypt:");
    // LOG_HEXDUMP_INF(key, 16, "key to encrypt:");

    int result =
        bt_ccm_encrypt(key, nonce, (uint8_t*)raw_data, sizeof(struct bthome_raw_data), NULL, 0, enc_data_buffer, 4);
    if (result)
    {
        LOG_INF("bt_ccm_encrypt() failed: %d", result);
        return result;
    }

    // LOG_HEXDUMP_INF(enc_data_buffer, sizeof(struct bthome_raw_data) + 4, "encrypted data + mic:");

    memcpy(&(payload->encrypted_data), enc_data_buffer, sizeof(struct bthome_raw_data));
    memcpy(&(payload->mic), enc_data_buffer + sizeof(struct bthome_raw_data), 4);

    // LOG_HEXDUMP_INF(&(payload->encrypted_data), sizeof(struct bthome_raw_data), "payload encrypted_data:");
    // LOG_HEXDUMP_INF(&(payload->counter), 4, "payload counter:");
    // LOG_HEXDUMP_INF(&(payload->mic), 4, "payload mic:");

    return result;
}

// static void bt_ready(int result)
// {
//     if (result)
//     {
//         LOG_ERR("bluetooth init failed: %d", result);
//         return;
//     }

//     // bt_le_ext_adv_create(ADV_PARAM, NULL, &adv_set_ptr);
//     // if (result)
//     // {
//     //     LOG_ERR("bluetooth extadvertising failed to create adv set: %d", result);
//     //     return;
//     // }

//     // result = bt_le_ext_adv_set_data(adv_set_ptr, advertising_data, ARRAY_SIZE(advertising_data), NULL, 0);
//     // if (result)
//     // {
//     //     LOG_ERR("bluetooth extadvertising failed to set data: %d, sizeof(advertising_payload):%d", result,
//     //     sizeof(advertising_payload)); return;
//     // }

//     // result = bt_le_ext_adv_start(adv_set_ptr, BT_LE_EXT_ADV_START_DEFAULT);
//     // if (result)
//     // {
//     //     LOG_ERR("bluetooth extadvertising failed to start: %d", result);
//     //     return;
//     // }

//     // result = bt_le_adv_start(
//     //     ADV_PARAM, advertising_data, ARRAY_SIZE(advertising_data), scan_response_data,
//     //     ARRAY_SIZE(scan_response_data));
//     // if (result)
//     // {
//     //     LOG_ERR(
//     //         "bluetooth adsr failed to set data: %d, sizeof(advertising_payload):%d",
//     //         result,
//     //         sizeof(advertising_payload));
//     //     return;
//     // }
//     // LOG_INF("bluetooth advertising started");
// }
K_THREAD_STACK_DEFINE(bluetooth_thread_stack, 2048);
struct k_thread bluetooth_thread;

void bluetooth_handle(void* p1, void* p2, void* p3)
{
    LOG_INF("enabling bluetooth");
    int result = bt_enable(NULL);
    if (result)
    {
        LOG_ERR("bluetooth init failed: %d", result);
        return;
    }
    else
    {
        LOG_INF("bluetooth ready");
    }
    result = encrypt_init();
    if (result)
    {
        LOG_ERR("encryption initialization failed: %d", result);
        return;
    }

    static size_t timeout_counter = 0;
    while (1)
    {
        int should_immdately_adv = 0;
        if (abs(raw_data.temperature - historical_temperature) >= 30) // 0.3 degree C
        {
            historical_temperature = raw_data.temperature;
            should_immdately_adv   = 1;
            LOG_INF("significant temperature change detected(peak): %hd", raw_data.temperature);
        }
        if (abs(raw_data.temperature - perv_temperature) >= 20) // 0.2 degree C
        {
            should_immdately_adv = 1;
            LOG_INF("significant temperature change detected(slope): %hd", raw_data.temperature);
        }
        if (abs(raw_data.humidity - historical_humidity) >= 50) // 0.5% RH
        {
            historical_humidity  = raw_data.humidity;
            should_immdately_adv = 1;
            LOG_INF("significant humidity change detected(peak): %hu", raw_data.humidity);
        }
        if (abs(raw_data.humidity - perv_humidity) >= 25) // 0.25% RH
        {
            should_immdately_adv = 1;
            LOG_INF("significant humidity change detected(slope): %hu", raw_data.humidity);
        }

        if (should_immdately_adv)
        {
            LOG_INF("immediate advertising triggered(rate of change)");
            int result = bt_le_adv_start(
                ADV_PARAM,
                advertising_data,
                ARRAY_SIZE(advertising_data),
                scan_response_data,
                ARRAY_SIZE(scan_response_data));
            if (result)
            {
                LOG_ERR(
                    "bluetooth adsr failed to set data: %d, sizeof(advertising_payload):%d",
                    result,
                    sizeof(advertising_payload));
                continue;
            }
            LOG_INF("bluetooth advertising started");
            for (size_t i = 0; i < 30; i++)
            {
                result = encrypt_payload(&advertising_payload, &raw_data, encryption_nonce, predefined_key);
                if (result)
                {
                    LOG_ERR("payload encryption failed: %d", result);
                    break;
                }
                result = bt_le_adv_update_data(
                    advertising_data, ARRAY_SIZE(advertising_data), scan_response_data, ARRAY_SIZE(scan_response_data));
                if (result)
                {
                    LOG_ERR("bluetooth adsr update failed to set data: %d", result);
                    break;
                }
                advertising_payload.counter++;
                LOG_INF("bluetooth advertising updated (%d/30)", i + 1);
                k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL)));
            }
            k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL * 2)));

            bt_le_adv_stop();
            LOG_INF("bluetooth advertising stopped");
        }
        else
        {
            timeout_counter = timeout_counter + 1;
            if (timeout_counter > 99)
            {
                LOG_INF("immediate advertising triggered(timeout)");
                timeout_counter = 0;
                int result      = bt_le_adv_start(
                    ADV_PARAM,
                    advertising_data,
                    ARRAY_SIZE(advertising_data),
                    scan_response_data,
                    ARRAY_SIZE(scan_response_data));
                if (result)
                {
                    LOG_ERR(
                        "bluetooth adsr failed to set data: %d, sizeof(advertising_payload):%d",
                        result,
                        sizeof(advertising_payload));
                    continue;
                }
                LOG_INF("bluetooth advertising started");
                for (size_t i = 0; i < 10; i++)
                {
                    result = encrypt_payload(&advertising_payload, &raw_data, encryption_nonce, predefined_key);
                    if (result)
                    {
                        LOG_ERR("payload encryption failed: %d", result);
                        break;
                    }
                    result = bt_le_adv_update_data(
                        advertising_data,
                        ARRAY_SIZE(advertising_data),
                        scan_response_data,
                        ARRAY_SIZE(scan_response_data));
                    if (result)
                    {
                        LOG_ERR("bluetooth adsr update failed to set data: %d", result);
                        break;
                    }
                    advertising_payload.counter++;
                    LOG_INF("bluetooth advertising updated (%d/10)", i + 1);
                    k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL)));
                }
                k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL * 2)));

                bt_le_adv_stop();
                LOG_INF("bluetooth advertising stopped");
            }
            else
            {
                k_sleep(K_MSEC(BT_TICK_TO_MSEC(BT_ADV_MIN_INTERVAL)));
            }
        }
    }
}

int main(void)
{
    LOG_INF("Alan nRF52832SHT40 BTHomev2 Sensor");

    LOG_INF("creating sensor thread");
    k_tid_t sensor_tid = k_thread_create(
        &sensor_thread,
        sensor_thread_stack,
        K_THREAD_STACK_SIZEOF(sensor_thread_stack),
        sensor_handle,
        NULL,
        NULL,
        NULL,
        1,
        0,
        K_NO_WAIT);
    k_thread_name_set(sensor_tid, "sensor_measurement");
    k_thread_start(sensor_tid);

    LOG_INF("creating bluetooth thread");
    k_tid_t bluetooth_tid = k_thread_create(
        &bluetooth_thread,
        bluetooth_thread_stack,
        K_THREAD_STACK_SIZEOF(bluetooth_thread_stack),
        bluetooth_handle,
        NULL,
        NULL,
        NULL,
        0,
        0,
        K_NO_WAIT);
    k_thread_name_set(bluetooth_tid, "bluetooth_advertising");
    k_thread_start(bluetooth_tid);

    (void)sensor_tid;
    (void)bluetooth_tid;

    // result = encrypt_payload(&advertising_payload, &raw_data, encryption_nonce, predefined_key);
    // if (result)
    // {
    //     LOG_ERR("payload encryption failed: %d", result);
    //     return -1;
    // }
    // result = bt_le_adv_update_data(
    //     advertising_data, ARRAY_SIZE(advertising_data), scan_response_data, ARRAY_SIZE(scan_response_data));
    // if (result)
    // {
    //     LOG_ERR("bluetooth adsr update failed to set data: %d", result);
    //     return -1;
    // }
    // advertising_payload.counter++;

    //     for (;;)
    //     {
    //         //
    //         // determine if significant change occurred
    //         //

    //         static adv_counter = 5;
    //     }

    //     //
    //     // encrypt and send adv
    //     //

    //     k_sleep(K_MSEC(0x4000 * 3)); // in the bad environment, extend adv time to increase chance of reception

    // next:
    //     k_sleep(K_MSEC(0x3000));
    // }
    // while (1)
    // {
    //     k_sleep(K_MSEC(1000 * 1000));
    // }
    return 0;
}