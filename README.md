# nRF52832 BTHome Sensor

<!-- ## SHT4x
- Sample SHT4x every 1.2s

## Bluetooth
- Send adv every 1s when actived
- When the tempearature changed 0.3degC from last change and 0.2degC from pervious sample, or humidity changed 0.5%RH from
last change and 0.25%RH from pervious sample, send 30 advs.
- When there are 300s no adv was sent, send 10 advs.

## Battery
- Sample battery by SAADC 300s
- The 100% level is defined as 3200mV, 0% defined as 2500mV -->

### Theory of Operation
- The whole main loop running periodicly which is between 5140ms and 6425ms. (all based on apple suggested 1285ms)
<!-- - When the tempearature changed 0.3degC from last significant change and 0.2degC from pervious sample, or humidity changed 0.5%RH from last significant change and 0.25%RH from pervious sample, the period shorten into 2570ms and 3855ms, lasts for 20 advs. (all based on apple suggested 1285ms) -->
- Battery is sampled every 600s, 128X oversampled, 12bit resolution, The 100% level is defined as 3200mV, 0% defined as 2500mV
- The paring key is stored in UICR_CUSTOM[15:0], default key is 8ba591a5ef8fd59990316d38e04ae9ed when the UICR is ffffffffffffffffffffffffffffffff, you can either program the UICR by "nrfjprog --memwr 0x10001304 --val 8ba591a5" or directly patch the bin
- SHT4x is working in high resloution mode, heater disabled