# nRF52832 BTHome Sensor

## SHT4x
- Sample SHT4x every 1.2s

## Bluetooth
- Send adv every 1s when actived
- When the tempearature changed 0.3degC from last change and 0.2degC from pervious sample, or humidity changed 0.5%RH from
last change and 0.25%RH from pervious sample, send 30 advs.
- When there are 300s no adv was sent, send 10 advs.

## Battery
- Sample battery by SAADC 300s
- The 100% level is defined as 3200mV, 0% defined as 2500mV