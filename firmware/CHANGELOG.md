# [Virida Air Quality Sensor Module]
## Virida Air Quality Sensor Module - detects and measures particulate (PM2.5) and VOC gasses level in the air.

----
**Firmware Development Beta 0.1.0-beta5 (12/16/2017)**
```
    - Features implemented:
        Added status payload HDR.
        Added blinking effect to alert high level PM2.5 or VOC to monitoringLEDEffect method.
        Added calibration status ble tx payload.
        Allowed user to select which sensor to calibrate.
        Enabled dock serial communication.
        Put a max value on LED power consumption.
        BLE transmit out docked/not docked in info payload.
    - Changes:
        Code cleanups.
    - Bug fixes
```
**Firmware Development Beta 0.1.0-beta4 (09/14/2017)**
```
    - Features implemented:
        When linked, module send out going to sleep or waking up HDR.
        When exiting sleep mode, PWR status led will blink yellow.
    - Changes:
        Refactored displayLEDEffect method to separate wakingUpLEDEffect, goingToSleepLEDEffect, monitoringLEDEffect, and blinkingLEDEffect.
        Code cleanups.
    - Bug fixes
```
**Firmware Development Beta 0.1.0-beta3 (08/10/2017)**
```
    - Features implemented:
        When linked, module send out going to sleep or waking up HDR.
        When exiting sleep mode, PWR status led will blink yellow.
    - Changes:
        Refactored displayLEDEffect method to separate wakingUpLEDEffect, goingToSleepLEDEffect, monitoringLEDEffect, and blinkingLEDEffect.
        Code cleanups.
    - Bug fixes
```
**Firmware Development Beta 0.1.0-beta2 (08/2/2017)**
```
    - Initial development commit
    - Features implemented:
        PM2.5 sensor sampling (with EMA) and calibration
        VOC gas sensor sampling (with EMA) and calibration
        Temp & humidity sampling (with EMA)
        BLE link
        Serial dock station link
        Battery level reading
        Capacitive touch on/off (sleep mode) button
```
