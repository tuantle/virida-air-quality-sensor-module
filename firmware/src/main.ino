/**
 * Copyright (c) 2017-present, Virida, Inc. All rights reserved.
 *
 *------------------------------------------------------------------------
 *
 * MODULE - Virida Air Quality Sensor (AQS) Module
 * DESCRIPTION -  Detects and measures particulate (PM2.5) and VOC gasses level in the air.
 *
 * AUTHOR - Tuan Le (tuan.t.lei@gmail.com)
 *
 *------------------------------------------------------------------------
 */

/* Libraries */
#include <math.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Sleep_n0m1.h>
#include <Adafruit_BLE_UART.h>
#include "FastLED.h"
#include "MillisTimer.h"
#include <dht11.h>

/* Global constants */
#define __DEBUG__                                                       true
#define __RUN_CALIBRATION_WHEN_WAKING_UP__                              false

#define AQS_FIRMWARE_VER                                               "0.1.0-beta5"
#define AQS_REV                                                        "P2"
#define AQS_SKEW                                                       "M"
#define AQS_MFN_SERIAL                                                 0
#define AQS_BLE_DEVICE_ALIAS                                           "VIRIDA"      // 7 characters max!

#define SLEEP_MSTATE                                                    0x01        // aqs is in power saving sleep state
#define WAKING_UP_MSTATE                                                0x02        // aqs is out of sleep state
#define MONITOR_MSTATE                                                  0x04        // aqs is awake and in periodic mornitoring state
#define LINKED_MSTATE                                                   0x08        // aqs is awake and is connected with app via bluetooth
#define SENSOR_CALIBRATION_MSTATE                                       0x10        // aqs is awake and doing sesnor calibration
#define SAMPLING_MSTATE                                                 0x20        // aqs is awake and taking samples fram sensors
#define DOCKED_MSTATE                                                   0x40        // aqs is on a dock station

#define DHT_SENSOR_READING_MERROR                                       0xF1
#define VOC_SENSOR_READING_MERROR                                       0xF2
#define PM25_SENSOR_READING_MERROR                                      0xF3
#define VOC_SENSOR_CALIBRATION_MERROR                                   0xF4
#define PM25_SENSOR_CALIBRATION_MERROR                                  0xF5
#define INCORRECT_PAYLOAD_SIZE_MERROR                                   0xF6
#define SAMPLE_COUNT_MERROR                                             0xF7
#define MONITOR_INTERVAL_MERROR                                         0xF8
// #define TX_PAYLOAD_BUFF_OVERFLOW_MERROR                                0xF9
#define RX_PAYLOAD_BUFF_OVERFLOW_MERROR                                 0xFA
#define UNKNOW_PAYLOAD_HDR_MERROR                                       0xFB

#define PAYLOAD_ENTERING_SLEEP_STATE_HDR                                0x10
#define PAYLOAD_EXITING_SLEEP_STATE_HDR                                 0x1A
#define PAYLOAD_WHOAMI_HDR                                              0x20
#define PAYLOAD_INFO_HDR                                                0x30
#define PAYLOAD_STATUS_HDR                                              0x3A
#define PAYLOAD_GENERAL_SETTING_HDR                                     0x40
#define PAYLOAD_UPDATE_GENERAL_SETTING_HDR                              0x4A
#define PAYLOAD_CALIBRATION_SETTING_HDR                                 0x4B
#define PAYLOAD_UPDATE_CALIBRATION_SETTING_HDR                          0x4C
#define PAYLOAD_START_CALIBRATION_HDR                                   0x50
#define PAYLOAD_CALIBRATION_STARTED_HDR                                 0x5A
#define PAYLOAD_CALIBRATION_ENDED_HDR                                   0x5B
#define PAYLOAD_CALIBRATION_STATUS_HDR                                  0x5C
#define PAYLOAD_START_SUBSCRIPTION_HDR                                  0x60
#define PAYLOAD_STOP_SUBSCRIPTION_HDR                                   0x6A
#define PAYLOAD_SUBSCRIPTION_STARTED_HDR                                0x6B
#define PAYLOAD_SUBSCRIPTION_ENDED_HDR                                  0x6C
#define PAYLOAD_SAMPLING_STARTED_HDR                                    0x70
#define PAYLOAD_SAMPLING_ENDED_HDR                                      0x7A
#define PAYLOAD_MERROR_CODE_HDR                                         0xF0

#define TXRX_PAYLOAD_BUFF_SIZE                                          20

#define MAX_LED_PWR_CONSUMPTION_MW                                      80
#define DEFAULT_LED_BRIGHTNESS_LEVEL                                    20
#define MLED_COUNT                                                      13             // number of leds of the neopixel

#define EPSILON                                                         0.009f

#define VSYS_MV                                                         5000          // aqs system voltage (mv)
#define EMA_DAMPING_FACTOR_NORMAL                                       0.8f
#define EMA_DAMPING_FACTOR_HIGH                                         0.4f
#define PM25_LVL_NORMAL_DAMPING_THRESHOLD                               6.0f
#define VOC_LVL_NORMAL_DAMPING_THRESHOLD                                0.5f

#define PM25_SENSOR_ILED_STARTUP_DELAY_US                               280
#define PM25_SENSOR_ILED_SHUTDOWN_DELAY_US                              40

#define PM25_SENSOR_CURVE_COEFFICIENT                                   0.16589f
#define PM25_SENSOR_MIN_LOW_DUST_LVL_MV                                 200
#define PM25_SENSOR_MAX_LOW_DUST_LVL_MV                                 1475
#define PM25_SENSOR_TYP_LOW_LVL_MV                                      600           // Sharp GP2Y1010AU0F typical low dust voltage lvl (mV)
#define PM25_SENSOR_DEFAULT_LOW_LVL_TARGET_THRESHOLD                    0.5f

#define VOC_SENSOR_CO_CURVE_COEFFICIENT_A                               4.3362f       // MICS5524 co gas curve coefficient a for y = ax^b
#define VOC_SENSOR_CO_CURVE_COEFFICIENT_B                               -1.1851f      // MICS5524 co gas curve coefficient b for y = ax^b
#define VOC_SENSOR_RLOAD                                                10000l        // MICS5524 RLoad = 10KOhm
#define VOC_SENSOR_MIN_RO                                               100000l       // MICS5524 min ro in Ohm
#define VOC_SENSOR_MAX_RO                                               1500000l       // MICS5524 max ro in Ohm
#define VOC_SENSOR_TYP_RO                                               400000l       // MICS5524 typical ro in Ohm
#define VOC_SENSOR_NATURAL_CO_PPM                                       0.2f          // natural atmospheric co ppm level for calibration purposes;
#define VOC_SENSOR_NATURAL_CO_PPM_TARGET_WINDOW                         0.05f

#define BETWEEN_CALIBRATION_CYCLE_DELAY_S                               4
#define VOC_SENSOR_WARMUP_DELAY_S                                       60            // 60s delay when aqs wake up to stabalize MICS5524 sesnors
#define WAKE_UP_DELAY_S                                                 30

#define PM25_SENSOR                                                     0
#define VOC_SENSOR                                                      1
#define PM25_SENSOR_CALIBRATION_FINE_STEP_SIZE_MV                       1             // 1 mV fine step size when calibrating GP2Y1010AU0F PM2.5
#define PM25_SENSOR_CALIBRATION_COARSE_STEP_SIZE_MV                     5             // 5 mV coarse step size when calibrating GP2Y1010AU0F PM2.5

#define VOC_SENSOR_CALIBRATION_FINE_STEP_SIZE_OHM                       250           // 250 Ohm fine step size when calibrating MICS5524 VOC
#define VOC_SENSOR_CALIBRATION_COARSE_STEP_SIZE_OHM                     5000          // 5K Ohm coarse step size when calibrating MICS5524 VOC

#define CALIBRATION_MAX_CYCLE_COUNT                                     8
#define CALIBRATION_MAX_OSCILLATION_COUNT                               4
#define CALIBRATION_MAX_ITERATION_COUNT                                 64

#define MAX_MONITORING_INTERVAL_S                                       60
#define MIN_MONITOR_IDLE_COUNT                                          24
#define MIN_SAMPLE_COUNT                                                4              // minimun number of sample points to reduce noise
#define MAX_SAMPLE_COUNT                                                16             // maximun number of sample points to reduce noise
#define TYP_SAMPLE_COUNT                                                8             // maximun number of sample points to reduce noise
#define DOCKING_DONE_COUNT                                              VOC_SENSOR_WARMUP_DELAY_S
#define GOING_TO_SLEEP_MAX_TOUCH_COUNT                                  5
#define WAKING_UP_MAX_TOUCH_COUNT                                       10
#define SHOW_WHOAMI_MAX_COUNT                                           3

/*------------------------------------------------------------------------
/* Global I/O PINs defined */
#define TOUCH_INTRPT_D                                                  3             // touch pad signal interupt digital input

#define MLEDS_D                                                         4

#define DOCKING_DET_NOT_D                                               5

#define BLE_REQ_D                                                       10
#define BLE_RDY_D                                                       2             // ble ready signal interupt digital input
#define BLE_RST_D                                                       9

#define DHT_SENSOR_D                                                    6
#define VOC_SENSOR_EN_NOT_D                                             8             // enable the MICS5524 voc sensor for reading
#define PM25_SENSOR_EN_NOT_D                                            7             // enable the gp2y1010au0f pm25 sensor for reading

#define PM25_SENSOR_VOUT_A                                              A0            // pm25 sensor reading analog output
#define VOC_SENSOR_VOUT_A                                               A3            // voc sensor reading analog output
#define BATTERY_LVL_A                                                   A1            // battery level reading analog input
#define BATTERY_TEMP_A                                                  A2            // battery temp reading analog input
/*
    http://vista.cira.colostate.edu/Improve/the-improve-algorithm/
    http://vista.cira.colostate.edu/Improve/wp-content/uploads/2016/03/IMPROVE_site_frh_2015.xls
    Using the average of fRHs and fRHl
*/
const int16_t F_RH_COMPENSATION_CURVE_PTS[] = {
    135, 136, 138, 140, 141, 143, 144, 146, 148, 150,
    151, 153, 156, 157, 159, 161, 163, 165, 168, 170,
    172, 174, 177, 179, 182, 184, 187, 190, 193, 196,
    200, 203, 207, 211, 215, 219, 223, 228, 234, 239,
    245, 251, 259, 266, 274, 284, 294, 306, 319, 333,
    350, 370, 393, 422//, 456, 500, 557, 635, 746
};
/*------------------------------------------------------------------------
/* Global variables */
Sleep sleep;
CRGB mleds[MLED_COUNT];
Adafruit_BLE_UART ble_uart = Adafruit_BLE_UART(BLE_REQ_D, BLE_RDY_D, BLE_RST_D);
dht11 dht;
MillisTimer periodic_task_runner = MillisTimer();

uint8_t mstate = 0x00;
uint8_t prev_mstate = mstate;
aci_evt_opcode_t ble_state;
uint8_t merror_code = 0x00;
uint8_t rx_pkg_buff_byte_count = 0;
uint8_t tx_pkg_buff_byte_count = 0;
uint8_t rx_pkg_buff[TXRX_PAYLOAD_BUFF_SIZE];
uint8_t tx_pkg_buff[TXRX_PAYLOAD_BUFF_SIZE];
/*------------------------------------------------------------------------
/* Function declarations */
bool hasMStateChanged (void);
bool isMState (uint16_t current_mstate);
void setMState (uint16_t new_mstate);
void unsetMState (uint16_t current_mstate);
bool isDocked (void);
void wakingUpLEDEffect (uint32_t duration_ms);
void goingToSleepLEDEffect (uint32_t duration_ms);
void monitoringLEDEffect (
   uint32_t duration_ms,
   uint32_t next_alert_color,
   bool blink
);
void blinkingLEDEffect (
   uint32_t duration_ms,
   uint32_t blink_color
);
void whoamiLEDEffect (
   uint32_t duration_ms
);
float emaFilter (
    float ema_value,
    float sample_value,
    float alpha
);
uint8_t readBatteryLvl (void);
// uint8_t readBatteryTemp (void);
void readTH (float* temp, float* humidity);
uint32_t readVOCRsValue (
    float temp,
    float humidity
);
float readPM25Lvl (
    uint16_t pm25_low_lvl_mv,
    float temp,
    float humidity
);
uint32_t calibrateVOCSensor (
    uint8_t ccycle_count,
    uint8_t sample_count,
    float voc_ref_co_ppm,
    float voc_ref_co_ppm_target_window
);
uint16_t calibratePM25Sensor (
    uint8_t ccycle_count,
    uint8_t sample_count,
    float pm25_low_lvl_target_threshold
);
void writeGeneralSettingToEEPROM(
    uint8_t led_brightness_lvl,
    uint8_t sample_count,
    uint8_t minterval_s
);
void readGeneralSettingFromEEPROM(
    uint8_t* led_brightness_lvl,
    uint8_t* sample_count,
    uint8_t* minterval_s
);
void writeSensorCalibrationSettingToEEPROM(
    uint16_t cDatestamp,
    uint16_t pm25_low_lvl_mv,
    float pm25_low_lvl_target_threshold,
    uint32_t voc_ro,
    float voc_ref_co_ppm,
    float voc_ref_co_ppm_target_window
);
void readSensorCalibrationSettingFromEEPROM(
    uint16_t* cDatestamp,
    uint16_t* pm25_low_lvl_mv,
    float* pm25_low_lvl_target_threshold,
    uint32_t* voc_ro,
    float* voc_ref_co_ppm,
    float* voc_ref_co_ppm_target_window
);
bool sensorSampling (
    uint8_t sample_count,
    uint16_t pm25_low_lvl_mv,
    uint32_t voc_ro,
    float temp,
    float humidity,
    float* pm25_lvl,
    float* voc_lvl
);
bool onTransmiting (
    uint8_t tx_pkg_hdr,
    uint8_t* tx_pkg_buff,
    uint8_t tx_pkg_buff_byte_count
);
uint8_t onReceiving (
    uint8_t* rx_pkg_buff,
    uint8_t* rx_pkg_buff_byte_count
);
void periodicTaskHandler (MillisTimer& ms);
void runApp (void);
void setup (void);
void loop (void);
/*------------------------------------------------------------------------
 * DESCRIPTION - Check that mstate has changed
 *
 * FUNCTION: hasMStateChanged
 * INPUTS:
 * OUTPUTS: has changed
 ------------------------------------------------------------------------*/
bool hasMStateChanged (void) {
    return prev_mstate != mstate;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Check current mstate
 *
 * FUNCTION: isMState
 * INPUTS:
 * OUTPUTS: is state equal
 ------------------------------------------------------------------------*/
bool isMState (uint16_t current_mstate) {
    return (mstate & current_mstate) == current_mstate;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Set new mstate
 *
 * FUNCTION: setMState
 * INPUTS: new_mstate
 ------------------------------------------------------------------------*/
void setMState (uint16_t new_mstate) {
    mstate = mstate | new_mstate;

    #if __DEBUG__
        // Serial.print("Set MState: "); Serial.println(new_mstate, HEX);
    #endif
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Unset current mstate
 *
 * FUNCTION: unsetMState
 * INPUTS: current_mstate
 ------------------------------------------------------------------------*/
void unsetMState (uint16_t current_mstate) {

    if (isMState(current_mstate)) {
        mstate = mstate ^ current_mstate;

        #if __DEBUG__
            // Serial.print("Unset MState: "); Serial.println(current_mstate, HEX);
        #endif
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Exponential moving average filter
 *
 * FUNCTION: emaFilter
 * INPUTS: ema_value
 *         sampled_value
 *         alpha
 * OUTPUTS: ema value
 ------------------------------------------------------------------------*/
float emaFilter (float ema_value, float sampled_value, float alpha) {
    return (sampled_value * alpha) + ((1 - alpha) * ema_value);
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Detecting if aqs is being docked
 *
 * FUNCTION: isDocked
 * INPUTS:
 * OUTPUTS: is docked or not
 ------------------------------------------------------------------------*/
bool isDocked (void) {
    // return digitalRead(DOCKING_DET_NOT_D) == LOW;
    return true;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Display RGB LEDs effects to indicate aqs waking up.
 *
 * FUNCTION: wakingUpLEDEffect
 * INPUTS: duration_ms - effect duration in ms
 ------------------------------------------------------------------------*/
 void wakingUpLEDEffect (
    uint32_t duration_ms
 ) {
    /* display aqs waking up effect */
    fill_solid(mleds, MLED_COUNT, CRGB::Black);
    for (uint8_t i = 0; i < MLED_COUNT; i++) {
        mleds[i] = CRGB::DeepSkyBlue;
        FastLED.show();
        FastLED.delay((2520 * duration_ms) >> 15);
    }
 }
 /*------------------------------------------------------------------------
  * DESCRIPTION - Display RGB LEDs effects to indicate aqs going to sleep.
  *
  * FUNCTION: goingToSleepLEDEffect
  * INPUTS: duration_ms - effect duration in ms
  ------------------------------------------------------------------------*/
 void goingToSleepLEDEffect (
    uint32_t duration_ms
 ) {
    /* display aqs going to sleep effect */
    for (uint8_t i = 0; i < MLED_COUNT; i++) {
        mleds[i] = CRGB::Black;
        FastLED.show();
        FastLED.delay((2520 * duration_ms) >> 15);
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Display RGB LEDs effects to indicate monitoring alert color.
 *
 * FUNCTION: monitoringLEDEffect
 * INPUTS: duration_ms - effect duration in ms
 *         next_alert_color - next pm2.5 or voc level alert color to display
 *         blink - blink effect for alert notification
 ------------------------------------------------------------------------*/
void monitoringLEDEffect (
   uint32_t duration_ms,
   uint32_t next_alert_color,
   bool blink
) {
    static uint32_t current_alert_color = CRGB::Black;

    for(uint8_t i = 0; i < 8; i++) {
        if (current_alert_color != next_alert_color) {
            fill_solid(mleds, MLED_COUNT - 1, blend(CRGB(current_alert_color), CRGB(next_alert_color), i * 36));
        } else if (blink) {
            if (i < 4) {
                fill_solid(mleds, MLED_COUNT - 1, blend(CRGB(current_alert_color), CRGB(CRGB::Black), i * 64));
            } else {
                fill_solid(mleds, MLED_COUNT - 1, blend(CRGB(CRGB::Black), CRGB(next_alert_color), i * 64));
            }
        }
        FastLED.show();
        FastLED.delay((8 * duration_ms) >> 6);
    }
    current_alert_color = next_alert_color;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Display RGB LEDs effects to blick power status led.
 *
 * FUNCTION: blinkingLEDEffect
 * INPUTS: duration_ms - effect duration in ms
 *         blick_color
 ------------------------------------------------------------------------*/
void blinkingLEDEffect (
   uint32_t duration_ms,
   // uint8_t blink_rate,
   uint32_t blink_color
) {
    for(uint8_t i = 0; i < 8; i++) {
        if (i < 4) {
            mleds[MLED_COUNT - 1] = blend(CRGB(blink_color), CRGB(CRGB::Black), i * 64);
        } else {
            mleds[MLED_COUNT - 1] = blend(CRGB(CRGB::Black), CRGB(blink_color), i * 64);
        }
        FastLED.show();
        FastLED.delay((8 * duration_ms) >> 6);
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Display RGB LEDs effects to show whoami.
 *
 * FUNCTION: whoamiLEDEffect
 * INPUTS: duration_ms - effect duration in ms
 ------------------------------------------------------------------------*/
void whoamiLEDEffect (
   uint32_t duration_ms
) {
    for(uint8_t i = 0; i < 8; i++) {
        if (i < 4) {
            fill_solid(mleds, MLED_COUNT - 1, blend(CRGB(CRGB::DeepSkyBlue), CRGB(CRGB::Black), i * 64));
        } else {
            fill_solid(mleds, MLED_COUNT - 1, blend(CRGB(CRGB::Black), CRGB(CRGB::DeepSkyBlue), i * 64));
        }
        FastLED.show();
        FastLED.delay((8 * duration_ms) >> 6);
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Read aqs battery level percentage.
 *               Covert to percentage batter percentage level = ((3.7V/1024) * adc) - 3.0) / 0.7 * 100
 *
 * FUNCTION: readBatteryLvl
 * OUTPUTS: battery level percentage
 ------------------------------------------------------------------------*/
uint8_t readBatteryLvl (void) {
    uint8_t batt_lvl = 0;
    uint16_t adc_value = analogRead(BATTERY_LVL_A);
    uint32_t batt_lvl_mv = ((uint32_t)adc_value * VSYS_MV) >> 10;

    if (batt_lvl_mv <= 3000) {
        batt_lvl = 0;
    } else if (batt_lvl_mv >= 3700) {
        batt_lvl = 100;
    } else {
        batt_lvl = (uint8_t)(round((batt_lvl_mv * 0.01f) - 30) * 14.286f);
    }

    return batt_lvl;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Read aqs battery temperature.
 *
 * FUNCTION: readBatteryTemp
 * OUTPUTS: battery temp in C
 ------------------------------------------------------------------------*/
// uint8_t readBatteryTemp (void) {
//     uint8_t batt_temp = 0;
//     uint16_t adc_value = analogRead(BATTERY_TEMP_A);
//     uint32_t batt_temp_mv = ((uint32_t)adc_value * VSYS_MV) >> 10;
//
//     // TODO: convert batt_temp_mv to batt_temp in C
//     return batt_temp;
// }
/*------------------------------------------------------------------------
 * DESCRIPTION - Read temperature value in celcius and relative humidity percentage
 *
 * FUNCTION: readTH
 * INPUTS: temperature in C
 *         humidity in percentage
 ------------------------------------------------------------------------*/
void readTH (float* temp, float* humidity) {
    if (dht.read(DHT_SENSOR_D) == DHTLIB_OK) {
        *temp =  dht.temperature;
        *humidity = dht.humidity;
    } else {
        merror_code = DHT_SENSOR_READING_MERROR;
        *temp =  0;
        *humidity = 0;
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Read VOC sensor resistenc (Rs). The Rs changes as the sensor
 *                is in the different consentration of the target VOC.
 *
 * FUNCTION: readVOCRsValue
 * INPUTS: temp - temperature in C
 *         humidity - relative humidity percentage
 * OUTPUTS: VOC sensor Rs (OHm) value of the sensor
 ------------------------------------------------------------------------*/
uint32_t readVOCRsValue (
    float temp,
    float humidity
) {
    uint16_t adc_value = 0;
    uint32_t adc_mv = 0;

    if (temp >= -30 && temp <= 85 && humidity <= 90) {
        adc_value = analogRead(VOC_SENSOR_VOUT_A);
        adc_mv = ((uint32_t)adc_value * VSYS_MV) >> 10;

        if (adc_mv > 0) {
            return VOC_SENSOR_RLOAD * (uint32_t)((float)(VSYS_MV - adc_mv) / adc_mv);
        } else {
            return 0;
        }
    } else {
        merror_code = VOC_SENSOR_READING_MERROR;
        return 0;
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Read PM2.5 density data from the Sharp GP2Y1010AU0F.
 *
 * FUNCTION: readPM25Lvl
 * INPUTS: pm25_low_lvl_mv - low dust voltage level
 *         temp - temperature in C
 *         humidity - relative humidity percentage
 * OUTPUTS: dust density level in ug/m^3
 ------------------------------------------------------------------------*/
float readPM25Lvl (
    uint16_t pm25_low_lvl_mv,
    float temp,
    float humidity
) {
    uint8_t rh_index = 0;
    uint16_t adc_value = 0;
    uint32_t adc_mv = 0;

    if (temp >= -10 && temp <= 65 && humidity <= 90) {

        digitalWrite(PM25_SENSOR_EN_NOT_D, LOW);
        delayMicroseconds(PM25_SENSOR_ILED_STARTUP_DELAY_US);
        adc_value = analogRead(PM25_SENSOR_VOUT_A);
        delayMicroseconds(PM25_SENSOR_ILED_SHUTDOWN_DELAY_US);
        digitalWrite(PM25_SENSOR_EN_NOT_D, HIGH);

        adc_mv = ((uint32_t)adc_value * VSYS_MV) >> 10;

        if (adc_mv < pm25_low_lvl_mv) {
            return 0;
        } else {
            rh_index = (uint8_t)round(humidity);

            if (rh_index < 37) {
                return PM25_SENSOR_CURVE_COEFFICIENT * (float)(adc_mv - pm25_low_lvl_mv);
            } else {
                return  100 * (PM25_SENSOR_CURVE_COEFFICIENT * (float)(adc_mv - pm25_low_lvl_mv)) / (float)F_RH_COMPENSATION_CURVE_PTS[rh_index - 37];
            }
        }
    } else {
        merror_code = PM25_SENSOR_READING_MERROR;
        return 0;
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Collect PM2.5 dust density & VOC data samples from Sharp GP2Y1010AU0F & MICS5524.
 *
 * FUNCTION: sensorSampling
 * INPUTS:  sample_count - number of samples
 *          pm25_low_lvl_mv - the index number associate with low dust voltage level.
 *          voc_ro
 *          temp
 *          humidity
 *          pm25_lvl
 *          voc_lvl
 * OUTPUTS: done sampling
 ------------------------------------------------------------------------*/
 bool sensorSampling (
     uint8_t sample_count,
     uint16_t pm25_low_lvl_mv,
     uint32_t voc_ro,
     float temp,
     float humidity,
     float* pm25_lvl,
     float* voc_lvl
 ) {
    static bool done = false;
    static uint8_t counter = 0;
    static float pm25_lvl_sample_sum = 0;
    static uint32_t voc_rs_sample_sum = 0;

    if (sample_count >= MIN_SAMPLE_COUNT && sample_count <= MAX_SAMPLE_COUNT) {
        if (counter == 0) {
            done = false;
        }

        if (counter < sample_count) {
            if (isMState(DOCKED_MSTATE)) {
                voc_rs_sample_sum += readVOCRsValue(temp, humidity);
            }
            pm25_lvl_sample_sum += readPM25Lvl(
                pm25_low_lvl_mv,
                temp, humidity
            );

            counter++;
        }

        if (counter == sample_count) {
            if (isMState(DOCKED_MSTATE)) {
                *voc_lvl = (float)voc_rs_sample_sum / (float)(sample_count * voc_ro);
            } else {
                *voc_lvl = -1;
            }

            *pm25_lvl = pm25_lvl_sample_sum / (float)sample_count;

            done = true;
            counter = 0;
            pm25_lvl_sample_sum = 0;
            voc_rs_sample_sum = 0;
        }
    } else {
        done = true;
    }

    return done;
}
/*------------------------------------------------------------------------
* DESCRIPTION - Get the Ro resistant from the MICS5524 by calibrating to CO ppm.
*                Assuming that the sensor is in clean air!
*
* FUNCTION: calibrateVOCSensor
* INPUTS: ccycle_count
*         sample_count - number of sample per search step.
*         voc_ref_co_ppm - atmospheric CO natural level as reference
*         voc_ref_co_ppm_target_window
* OUTPUTS: Ro of the sensor   .
------------------------------------------------------------------------*/
uint32_t calibrateVOCSensor(
    uint8_t ccycle_count,
    uint8_t sample_count,
    float voc_ref_co_ppm,
    float voc_ref_co_ppm_target_window
) {
    float inv_sample_count = 1;
    uint32_t voc_rs_sample_sum = 0;
    float voc_co_ppm = 0;
    float voc_prev_co_ppm = 0;
    float voc_co_ppm_error = 0;
    float temp = 0;
    float humidity = 0;
    uint32_t voc_ro = VOC_SENSOR_TYP_RO;
    uint32_t voc_ro_step_size = VOC_SENSOR_CALIBRATION_COARSE_STEP_SIZE_OHM;
    uint32_t voc_ro_pts[CALIBRATION_MAX_CYCLE_COUNT];

    int8_t prev_slope = 0;
    int8_t slope = 0;
    uint8_t oscillation_counter = 0;
    uint8_t iteration_counter = 0;

    if (ccycle_count <= CALIBRATION_MAX_CYCLE_COUNT && sample_count >= MIN_SAMPLE_COUNT && sample_count <= MAX_SAMPLE_COUNT) {

        inv_sample_count = 1.0f / (float)sample_count;

        #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
            Serial.print("Searching Natural Atmospheric CO Ro @ "); Serial.print(voc_ro); Serial.println("Ohm");
        #endif

        for (uint8_t i = 0; i < sample_count; i++) {
            readTH(&temp, &humidity);
            voc_rs_sample_sum += readVOCRsValue(temp, humidity);
            blinkingLEDEffect(1000, CRGB::Green);
        }
        voc_co_ppm = VOC_SENSOR_CO_CURVE_COEFFICIENT_A * pow((voc_rs_sample_sum * inv_sample_count) / (float)voc_ro, VOC_SENSOR_CO_CURVE_COEFFICIENT_B);
        voc_co_ppm_error = abs(voc_co_ppm - voc_ref_co_ppm);

        for (uint8_t j = 0; j < ccycle_count; j++) {
            while (voc_co_ppm < (voc_ref_co_ppm - voc_ref_co_ppm_target_window) || voc_co_ppm > (voc_ref_co_ppm + voc_ref_co_ppm_target_window)) {
                if (voc_co_ppm > voc_ref_co_ppm + voc_ref_co_ppm_target_window) {
                    if ((voc_ro - voc_ro_step_size) >= VOC_SENSOR_MIN_RO) {
                        voc_ro -= voc_ro_step_size;

                        #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                            Serial.println(voc_co_ppm);
                            Serial.print("(-) @ "); Serial.print(voc_ro); Serial.print(", "); Serial.print(voc_ro_step_size); Serial.println("Ohm");
                        #endif
                    } else {
                        voc_ro = VOC_SENSOR_MIN_RO;
                        merror_code = VOC_SENSOR_CALIBRATION_MERROR;
                        voc_ro_pts[j] = voc_ro;
                        break;
                    }
                } else if (voc_co_ppm < voc_ref_co_ppm + voc_ref_co_ppm_target_window) {
                    if ((voc_ro + voc_ro_step_size) <= VOC_SENSOR_MAX_RO) {
                        voc_ro += voc_ro_step_size;

                        #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                            Serial.println(voc_co_ppm);
                            Serial.print("(+) @ "); Serial.print(voc_ro); Serial.print(", "); Serial.print(voc_ro_step_size); Serial.println("Ohm");
                        #endif
                    } else {
                        voc_ro = VOC_SENSOR_MAX_RO;
                        merror_code = VOC_SENSOR_CALIBRATION_MERROR;
                        voc_ro_pts[j] = voc_ro;
                        break;
                    }
                }
                voc_rs_sample_sum = 0;
                for (uint8_t i = 0; i < sample_count; i++) {
                    readTH(&temp, &humidity);
                    voc_rs_sample_sum += readVOCRsValue(temp, humidity);
                    blinkingLEDEffect(1000, CRGB::Green);
                }
                voc_prev_co_ppm = voc_co_ppm;
                voc_co_ppm = emaFilter(
                    voc_co_ppm,
                    VOC_SENSOR_CO_CURVE_COEFFICIENT_A * pow((voc_rs_sample_sum * inv_sample_count) / (float)voc_ro, VOC_SENSOR_CO_CURVE_COEFFICIENT_B),
                    EMA_DAMPING_FACTOR_NORMAL
                );

                if ((voc_co_ppm - voc_prev_co_ppm) > 0) {
                    prev_slope = slope;
                    slope = 1;
                } else if ((voc_co_ppm - voc_prev_co_ppm) < 0) {
                    prev_slope = slope;
                    slope = -1;
                } else {
                    prev_slope = slope;
                    slope = 0;
                }

                if (abs(slope - prev_slope) > EPSILON) {
                    oscillation_counter++;
                    if (oscillation_counter > CALIBRATION_MAX_OSCILLATION_COUNT) {
                        voc_ro_pts[j] = voc_ro;
                        break;
                    }
                }

                voc_co_ppm_error = abs(voc_co_ppm - voc_ref_co_ppm);
                if (voc_co_ppm_error <= (2 * voc_ref_co_ppm_target_window)) {
                    voc_ro_step_size = (205 * voc_ro_step_size) >> 11;
                } else {
                    voc_ro_step_size += (205 * voc_ro_step_size) >> 11;
                }

                if (voc_ro_step_size < VOC_SENSOR_CALIBRATION_FINE_STEP_SIZE_OHM) {
                    voc_ro_step_size = VOC_SENSOR_CALIBRATION_FINE_STEP_SIZE_OHM;
                } else if (voc_ro_step_size > VOC_SENSOR_CALIBRATION_COARSE_STEP_SIZE_OHM << 4) {
                    voc_ro_step_size = VOC_SENSOR_CALIBRATION_COARSE_STEP_SIZE_OHM << 4;
                }

                iteration_counter++;
                if (iteration_counter >= CALIBRATION_MAX_ITERATION_COUNT) {
                    merror_code = VOC_SENSOR_CALIBRATION_MERROR;
                    voc_ro = VOC_SENSOR_TYP_RO;
                    voc_ro_pts[j] = voc_ro;
                    break;
                }
            }

            oscillation_counter = 0;
            iteration_counter = 0;

            #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                Serial.print(j); Serial.print(" - Found VOC Ro @ "); Serial.print(voc_ro); Serial.println("Ohm");
            #endif

            if (isMState(LINKED_MSTATE)) {
                tx_pkg_buff_byte_count = 2;

                tx_pkg_buff[0] = (uint8_t)PM25_SENSOR;
                tx_pkg_buff[1] = (uint8_t)((float)(j / ccycle_count) * 100);

                if (onTransmiting(PAYLOAD_CALIBRATION_STATUS_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                    memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                    tx_pkg_buff_byte_count = 0;
                }
            }

            // blinkingLEDEffect(BETWEEN_CALIBRATION_CYCLE_DELAY_S * 1000, CRGB::Green);
            delay(BETWEEN_CALIBRATION_CYCLE_DELAY_S * 1000);
        }
    }

    voc_ro = 0;
    for (uint8_t j = 0; j < ccycle_count; j++) {
        voc_ro += voc_ro_pts[j];
    }
    voc_ro = (uint32_t)(round((float)voc_ro / (float)ccycle_count));

    #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
        Serial.print("Calibrated VOC Ro @ "); Serial.print(voc_ro); Serial.println("Ohm");
    #endif

    return voc_ro;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Searching for voltage level when the PM2.5 density data from the Sharp GP2Y1010AU0F is low or zero.
 *                Assuming that the sensor is in clean air!
 *
 * FUNCTION: calibratePM25Sensor
 * INPUTS: ccycle_count
 *         sample_count - number of sample per search step.
 *         pm25_low_lvl_target_threshold
 * OUTPUTS: int - low dust voltage level (mv)
------------------------------------------------------------------------*/
uint16_t calibratePM25Sensor (
    uint8_t ccycle_count,
    uint8_t sample_count,
    float pm25_low_lvl_target_threshold
) {
    float pm25_lvl_sample_sum = 0;
    float pm25_lvl = 0;
    float inv_sample_count = 1;
    float temp = 0;
    float humidity = 0;
    uint16_t pm25_low_lvl_mv = PM25_SENSOR_TYP_LOW_LVL_MV;
    uint16_t pm25_low_lvl_mv_pts[CALIBRATION_MAX_CYCLE_COUNT];
    uint16_t lvl_mv_step_size = PM25_SENSOR_CALIBRATION_COARSE_STEP_SIZE_MV;

    if (ccycle_count <= CALIBRATION_MAX_CYCLE_COUNT && sample_count >= MIN_SAMPLE_COUNT && sample_count <= MAX_SAMPLE_COUNT) {
        inv_sample_count = 1.0f / (float)sample_count;

        for (uint8_t i = 0; i < sample_count; i++) {
            readTH(&temp, &humidity);
            pm25_lvl_sample_sum += readPM25Lvl(pm25_low_lvl_mv, temp, humidity);
            blinkingLEDEffect(1000, CRGB::Green);
        }
        pm25_lvl = pm25_lvl_sample_sum * inv_sample_count;

        for (uint8_t j = 0; j < ccycle_count; j++) {
            #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                Serial.print("Searching Low Dust VLvl @ "); Serial.print(pm25_low_lvl_mv); Serial.println("mV");
            #endif

            if (pm25_lvl >= 0 && pm25_lvl <= pm25_low_lvl_target_threshold) {
                while (pm25_low_lvl_mv > PM25_SENSOR_MIN_LOW_DUST_LVL_MV) {
                    pm25_lvl_sample_sum = 0;
                    for (uint8_t i = 0; i < sample_count; i++) {
                        readTH(&temp, &humidity);
                        pm25_lvl_sample_sum += readPM25Lvl(pm25_low_lvl_mv, temp, humidity);
                        #if __DEBUG__
                            Serial.println(pm25_lvl_sample_sum);
                        #endif

                        blinkingLEDEffect(1000, CRGB::Green);
                    }
                    pm25_lvl = emaFilter(pm25_lvl, pm25_lvl_sample_sum * inv_sample_count, EMA_DAMPING_FACTOR_NORMAL);
                    if (pm25_lvl >= 0 && pm25_lvl <= pm25_low_lvl_target_threshold) {
                        if ((pm25_low_lvl_mv - lvl_mv_step_size) > PM25_SENSOR_MIN_LOW_DUST_LVL_MV) {
                            pm25_low_lvl_mv -= lvl_mv_step_size;
                            #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                                Serial.println(pm25_lvl);
                                Serial.print("(-) @ "); Serial.print(pm25_low_lvl_mv); Serial.println("mV");
                            #endif
                        } else {
                            pm25_low_lvl_mv = PM25_SENSOR_MIN_LOW_DUST_LVL_MV;
                            merror_code = PM25_SENSOR_CALIBRATION_MERROR;
                            pm25_low_lvl_mv_pts[j] = pm25_low_lvl_mv;
                            break;
                        }
                    } else {
                        pm25_low_lvl_mv_pts[j] = pm25_low_lvl_mv;
                        break;
                    }
                }
            } else {
                while (pm25_low_lvl_mv < PM25_SENSOR_MAX_LOW_DUST_LVL_MV) {
                    pm25_lvl_sample_sum = 0;
                    for (uint8_t i = 0; i < sample_count; i++) {
                        readTH(&temp, &humidity);
                        pm25_lvl_sample_sum += readPM25Lvl(pm25_low_lvl_mv, temp, humidity);
                        #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                            Serial.println(pm25_lvl_sample_sum);
                        #endif

                        blinkingLEDEffect(1000, CRGB::Green);
                    }
                    pm25_lvl = emaFilter(pm25_lvl, pm25_lvl_sample_sum * inv_sample_count, EMA_DAMPING_FACTOR_NORMAL);
                    if (pm25_lvl > pm25_low_lvl_target_threshold) {
                        if ((pm25_low_lvl_mv + lvl_mv_step_size) < PM25_SENSOR_MAX_LOW_DUST_LVL_MV) {
                            pm25_low_lvl_mv += lvl_mv_step_size;
                            #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                                Serial.println(pm25_lvl);
                                Serial.print("(+) @ "); Serial.print(pm25_low_lvl_mv); Serial.println("mV");
                            #endif
                        } else {
                            pm25_low_lvl_mv = PM25_SENSOR_MAX_LOW_DUST_LVL_MV;
                            merror_code = PM25_SENSOR_CALIBRATION_MERROR;
                            pm25_low_lvl_mv_pts[j] = pm25_low_lvl_mv;
                            break;
                        }
                    } else {
                        pm25_low_lvl_mv_pts[j] = pm25_low_lvl_mv;
                        break;
                    }
                }
            }

            lvl_mv_step_size = PM25_SENSOR_CALIBRATION_FINE_STEP_SIZE_MV;

            #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
                Serial.println(pm25_lvl);
                Serial.print(j); Serial.print(" - Found Low Dust VLvl @ "); Serial.print(pm25_low_lvl_mv_pts[j]); Serial.println("mV");
            #endif

            if (isMState(LINKED_MSTATE)) {
                tx_pkg_buff_byte_count = 2;

                tx_pkg_buff[0] = (uint8_t)VOC_SENSOR;
                tx_pkg_buff[1] = (uint8_t)((float)(j / ccycle_count) * 100);

                if (onTransmiting(PAYLOAD_CALIBRATION_STATUS_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                    memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                    tx_pkg_buff_byte_count = 0;
                }
            }

            // blinkingLEDEffect(BETWEEN_CALIBRATION_CYCLE_DELAY_S * 1000, CRGB::Green);
            delay(BETWEEN_CALIBRATION_CYCLE_DELAY_S * 1000);
        }
    }

    pm25_low_lvl_mv = 0;
    for (uint8_t j = 0; j < ccycle_count; j++) {
        pm25_low_lvl_mv += pm25_low_lvl_mv_pts[j];
    }
    pm25_low_lvl_mv = (uint16_t)(round((float)pm25_low_lvl_mv / (float)ccycle_count));

    #if __DEBUG__ && __RUN_CALIBRATION_WHEN_WAKING_UP__
        Serial.print("Calibrated Low Dust VLvl @ "); Serial.print(pm25_low_lvl_mv); Serial.println("mV");
    #endif

    return pm25_low_lvl_mv;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Write general setting data to eeprom.
 *
 * FUNCTION: writeGeneralSettingToEEPROM
 * INPUTS: led_brightness_lvl
 *         sample_count
 *         minterval_s
 ------------------------------------------------------------------------*/
void writeGeneralSettingToEEPROM(
    uint8_t led_brightness_lvl,
    uint8_t sample_count,
    uint8_t minterval_s
) {
    EEPROM.update(21, 3);

    EEPROM.update(22, led_brightness_lvl);
    EEPROM.update(23, sample_count);
    EEPROM.update(24, minterval_s);
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Read general setting data from eeprom.
 *
 * FUNCTION: readGeneralSettingFromEEPROM
 * INPUTS: led_brightness_lvl
 *         sample_count
 *         minterval_s
 ------------------------------------------------------------------------*/
void readGeneralSettingFromEEPROM(
    uint8_t* led_brightness_lvl,
    uint8_t* sample_count,
    uint8_t* minterval_s
) {
    if (EEPROM.read(21) == 3) {
        *led_brightness_lvl = EEPROM.read(22);
        *sample_count = EEPROM.read(23);
        *minterval_s = EEPROM.read(24);

    } else {
        *led_brightness_lvl = DEFAULT_LED_BRIGHTNESS_LEVEL;
        *sample_count = TYP_SAMPLE_COUNT;
        *minterval_s = TYP_SAMPLE_COUNT + MIN_MONITOR_IDLE_COUNT;
        writeGeneralSettingToEEPROM(
            *led_brightness_lvl,
            *sample_count,
            *minterval_s
        );
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Write calibration setting data to eeprom.
 *
 * FUNCTION: writeSensorCalibrationSettingToEEPROM
 * INPUTS: cDatestamp
 *         pm25_low_lvl_mv
 *         pm25_low_lvl_target_threshold
 *         voc_ro
 *         voc_ref_co_ppm
 *         voc_ref_co_ppm_target_window
 ------------------------------------------------------------------------*/
void writeSensorCalibrationSettingToEEPROM(
    uint16_t cDatestamp,
    uint16_t pm25_low_lvl_mv,
    float pm25_low_lvl_target_threshold,
    uint32_t voc_ro,
    float voc_ref_co_ppm,
    float voc_ref_co_ppm_target_window
) {
    EEPROM.update(0, 15);

    EEPROM.update(1, (uint8_t)(cDatestamp >> 8));
    EEPROM.update(2, (uint8_t)cDatestamp);

    EEPROM.update(3, (uint8_t)(pm25_low_lvl_mv >> 8));
    EEPROM.update(4, (uint8_t)pm25_low_lvl_mv);

    EEPROM.update(5, (uint8_t)((uint16_t)(pm25_low_lvl_target_threshold * 100) >> 8));
    EEPROM.update(6, (uint8_t)(uint16_t)(pm25_low_lvl_target_threshold * 100));

    EEPROM.update(7, (uint8_t)(voc_ro >> 24));
    EEPROM.update(8, (uint8_t)(voc_ro >> 16));
    EEPROM.update(9, (uint8_t)(voc_ro >> 8));
    EEPROM.update(10, (uint8_t)voc_ro);

    EEPROM.update(11, (uint8_t)((uint16_t)(voc_ref_co_ppm * 100) >> 8));
    EEPROM.update(12, (uint8_t)(uint16_t)(voc_ref_co_ppm * 100));

    EEPROM.update(13, (uint8_t)((uint16_t)voc_ref_co_ppm_target_window >> 8));
    EEPROM.update(14, (uint8_t)(uint16_t)(voc_ref_co_ppm_target_window * 100));
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Read calibration data from eeprom.
 *
 * FUNCTION: readSensorCalibrationSettingFromEEPROM
 * INPUTS: pm25_low_lvl_mv
  *        pm25_low_lvl_target_threshold
 *         voc_ro
 *         voc_ref_co_ppm
 *         voc_ref_co_ppm_target_window
 ------------------------------------------------------------------------*/
void readSensorCalibrationSettingFromEEPROM(
    uint16_t* cDatestamp,
    uint16_t* pm25_low_lvl_mv,
    float* pm25_low_lvl_target_threshold,
    uint32_t* voc_ro,
    float* voc_ref_co_ppm,
    float* voc_ref_co_ppm_target_window
) {
    if (EEPROM.read(0) == 15) {
        *cDatestamp = ((uint16_t)EEPROM.read(1) << 8) + (uint16_t)EEPROM.read(2);
        *pm25_low_lvl_mv = ((uint16_t)EEPROM.read(3) << 8) + (uint16_t)EEPROM.read(4);
        *pm25_low_lvl_target_threshold = (float)(
            ((uint32_t)EEPROM.read(5) << 8) +
            (uint32_t)EEPROM.read(6)
        ) / 100;
        *voc_ro = ((uint32_t)EEPROM.read(7) << 24) +
            ((uint32_t)EEPROM.read(8) << 16) +
            ((uint32_t)EEPROM.read(9) << 8) +
            (uint32_t)EEPROM.read(10);
        *voc_ref_co_ppm = (float)(
            ((uint32_t)EEPROM.read(11) << 8) +
            (uint32_t)EEPROM.read(12)
        ) / 100;
        *voc_ref_co_ppm_target_window = (float)(
            ((uint32_t)EEPROM.read(13) << 8) +
            (uint32_t)EEPROM.read(14)
        ) / 100;
    } else {
        *cDatestamp = 0;
        *pm25_low_lvl_mv = PM25_SENSOR_TYP_LOW_LVL_MV;
        *pm25_low_lvl_target_threshold = PM25_SENSOR_DEFAULT_LOW_LVL_TARGET_THRESHOLD;
        *voc_ro = VOC_SENSOR_TYP_RO;
        *voc_ref_co_ppm = VOC_SENSOR_NATURAL_CO_PPM; // natural atmospheric CO level for calibration purposes
        *voc_ref_co_ppm_target_window = VOC_SENSOR_NATURAL_CO_PPM_TARGET_WINDOW;
        writeSensorCalibrationSettingToEEPROM(
            *cDatestamp,
            *pm25_low_lvl_mv,
            *pm25_low_lvl_target_threshold,
            *voc_ro,
            *voc_ref_co_ppm,
            *voc_ref_co_ppm_target_window
        );
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - On receiving package data from serial 1 or bluetooth.
 *
 * FUNCTION: onReceiving
 * INPUTS: rx_pkg_buff
 *         rx_pkg_buff_byte_count
 * OUTPUTS: uint8_t - package header
 ------------------------------------------------------------------------*/
uint8_t onReceiving (
    uint8_t* rx_pkg_buff,
    uint8_t* rx_pkg_buff_byte_count
) {
    static bool rx_pkg_received = false;
    uint8_t rx_pkg_byte;
    static uint8_t rx_pkg_hdr = 0x00;
    static uint8_t rx_pkg_hdr_byte_count = 0;
    static uint8_t expected_rx_pkg_buff_byte_count = 0;

    rx_pkg_received = false;

    while (ble_state == ACI_EVT_CONNECTED && ble_uart.available()) {
        rx_pkg_byte = ble_uart.read();
    // while (Serial1.available() || (ble_state == ACI_EVT_CONNECTED && ble_uart.available())) {
    //     if (ble_state == ACI_EVT_CONNECTED && ble_uart.available() && !Serial1.available()) {
    //         rx_pkg_byte = ble_uart.read();
    //     } else if (!ble_uart.available() && Serial1.available()) {
    //         rx_pkg_byte = Serial1.read();
    //     }
        switch (rx_pkg_hdr_byte_count) {
            case 0:
                rx_pkg_hdr = rx_pkg_byte;
                rx_pkg_hdr_byte_count++;
                break;
            case 1:
                expected_rx_pkg_buff_byte_count = rx_pkg_byte;
                rx_pkg_hdr_byte_count++;
                if (expected_rx_pkg_buff_byte_count == 0) {
                    rx_pkg_received = true;
                    rx_pkg_hdr_byte_count = 0;
                }
                break;
            case 2:
                if (*rx_pkg_buff_byte_count < TXRX_PAYLOAD_BUFF_SIZE) {
                    rx_pkg_buff[*rx_pkg_buff_byte_count] = rx_pkg_byte;
                    (*rx_pkg_buff_byte_count)++;

                    if (*rx_pkg_buff_byte_count == expected_rx_pkg_buff_byte_count) {
                        rx_pkg_received = true;
                        rx_pkg_hdr_byte_count = 0;
                        expected_rx_pkg_buff_byte_count = 0;
                    }
                } else {
                    rx_pkg_received = false;
                    rx_pkg_hdr_byte_count = 0;
                    expected_rx_pkg_buff_byte_count = 0;
                }
                break;
            default:
                rx_pkg_received = false;
                rx_pkg_hdr_byte_count = 0;
                expected_rx_pkg_buff_byte_count = 0;
                break;
        }
    }

    if (rx_pkg_received) {
        return rx_pkg_hdr;
    } else {
        return 0x00;
    }
}
/*------------------------------------------------------------------------
 * DESCRIPTION - On transmitting package data to serial 1 or bluetooth.
 *
 * FUNCTION: onTransmiting
 * INPUTS: tx_pkg_hdr
 *         tx_pkg_buff
 *         tx_pkg_buff_byte_count
 * OUTPUTS: bool
 ------------------------------------------------------------------------*/
bool onTransmiting (
    uint8_t tx_pkg_hdr,
    uint8_t* tx_pkg_buff,
    uint8_t tx_pkg_buff_byte_count
) {
    static bool tx_pkg_transmitted = false;
    uint8_t tx_pkg_hdr_parts[2];

    tx_pkg_transmitted = false;
    tx_pkg_hdr_parts[0] = tx_pkg_hdr;
    tx_pkg_hdr_parts[1] = tx_pkg_buff_byte_count;

    if (ble_state == ACI_EVT_CONNECTED && !ble_uart.available()) {
        ble_uart.write(tx_pkg_hdr_parts, 2);
    }
    // if (Serial1.availableForWrite()) {
    //     Serial1.write(tx_pkg_hdr_parts, 2);
    // }

    if (tx_pkg_buff_byte_count != 0) {
        if (tx_pkg_buff_byte_count <= TXRX_PAYLOAD_BUFF_SIZE) {
            if (ble_state == ACI_EVT_CONNECTED && !ble_uart.available()) {
                ble_uart.write((uint8_t*)tx_pkg_buff, tx_pkg_buff_byte_count);
            }
            // if (Serial1.availableForWrite()) {
            //     Serial1.write((uint8_t*)tx_pkg_buff, tx_pkg_buff_byte_count);
            // }
        } else {
            for (uint8_t i = TXRX_PAYLOAD_BUFF_SIZE; i < tx_pkg_buff_byte_count - TXRX_PAYLOAD_BUFF_SIZE; i += TXRX_PAYLOAD_BUFF_SIZE) {
                if (TXRX_PAYLOAD_BUFF_SIZE - i <= TXRX_PAYLOAD_BUFF_SIZE) {
                    if (ble_state == ACI_EVT_CONNECTED && !ble_uart.available()) {
                        ble_uart.write((uint8_t*)tx_pkg_buff, TXRX_PAYLOAD_BUFF_SIZE - i);
                    }
                    // if (Serial1.availableForWrite()) {
                    //     Serial1.write((uint8_t*)tx_pkg_buff, TXRX_PAYLOAD_BUFF_SIZE - i);
                    // }
                } else {
                    if (ble_state == ACI_EVT_CONNECTED && !ble_uart.available()) {
                        ble_uart.write((uint8_t*)tx_pkg_buff, TXRX_PAYLOAD_BUFF_SIZE);
                    }
                    // if (Serial1.availableForWrite()) {
                    //     Serial1.write((uint8_t*)tx_pkg_buff, TXRX_PAYLOAD_BUFF_SIZE);
                    // }
                }
            }
        }
        tx_pkg_transmitted = true;
    }

    return tx_pkg_transmitted;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - Call back handler for periodic_task_runner. Called at regular time period.
 *
 * FUNCTION: periodicTaskHandler
 ------------------------------------------------------------------------*/
void periodicTaskHandler (MillisTimer& ms) {
    setMState(SAMPLING_MSTATE);
}
/*------------------------------------------------------------------------
 * DESCRIPTION - AQS main app.
 *
 * FUNCTION: runApp
 ------------------------------------------------------------------------*/
void runApp (void) {
    #if __DEBUG__
        static uint8_t loop_counter = 0;
    #endif

    static bool subscribed = false;
    static uint8_t show_whoami_counter = 0;
    static uint8_t touch_hold_counter = 0;
    static uint8_t docking_counter = 0;

    static uint8_t sample_count = TYP_SAMPLE_COUNT;
    static uint8_t minterval_s = TYP_SAMPLE_COUNT + MIN_MONITOR_IDLE_COUNT;

    static uint8_t led_brightness_lvl = DEFAULT_LED_BRIGHTNESS_LEVEL;
    static uint8_t batt_lvl = 0;

    static bool selected_sensor_for_calibration = PM25_SENSOR;
    static uint16_t cDatestamp;
    static uint16_t pm25_low_lvl_mv = PM25_SENSOR_TYP_LOW_LVL_MV;
    static float pm25_low_lvl_target_threshold = PM25_SENSOR_DEFAULT_LOW_LVL_TARGET_THRESHOLD;
    static uint32_t voc_ro = VOC_SENSOR_TYP_RO;
    static float voc_ref_co_ppm = VOC_SENSOR_NATURAL_CO_PPM;
    static float voc_ref_co_ppm_target_window = VOC_SENSOR_NATURAL_CO_PPM_TARGET_WINDOW;

    static uint8_t pm25_lvl_alert_index = 0;
    static uint8_t voc_lvl_alert_index = 0;
    static float pm25_lvl = 0;
    static float ema_pm25_lvl = -1;
    static float voc_lvl = -1;
    static float ema_voc_lvl = -1;
    static float temp = 0;
    static float ema_temp = -1;
    static float humidity = 0;
    static float ema_humidity = -1;

    uint32_t pm25_lvl_alert_color = CRGB::Lime;
    uint32_t voc_lvl_alert_color = CRGB::Lime;

    if (digitalRead(TOUCH_INTRPT_D) == HIGH && touch_hold_counter <= WAKING_UP_MAX_TOUCH_COUNT) {
        touch_hold_counter++;
    } else if (digitalRead(TOUCH_INTRPT_D) == LOW && touch_hold_counter > 0 && touch_hold_counter != GOING_TO_SLEEP_MAX_TOUCH_COUNT) {
        touch_hold_counter--;
    }

    if (touch_hold_counter == GOING_TO_SLEEP_MAX_TOUCH_COUNT) {
        if (isMState(LINKED_MSTATE)) {
            tx_pkg_buff_byte_count = 0;

            if (onTransmiting(PAYLOAD_ENTERING_SLEEP_STATE_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                tx_pkg_buff_byte_count = 0;
            }
        }

        mstate = 0x00; // reset aqs state to default
        setMState(SLEEP_MSTATE);

        periodic_task_runner.stop();

        digitalWrite(PM25_SENSOR_EN_NOT_D, HIGH); // turn off PM2.5 sensor
        digitalWrite(VOC_SENSOR_EN_NOT_D, HIGH); // turn off VOC sensor

        goingToSleepLEDEffect(1000);

        sleep.pwrDownMode();
        sleep.sleepPinInterrupt(TOUCH_INTRPT_D, CHANGE);
    }

    if (isMState(SLEEP_MSTATE)) {
        if (touch_hold_counter == WAKING_UP_MAX_TOUCH_COUNT) {
            touch_hold_counter = 0;

            unsetMState(SLEEP_MSTATE);
            setMState(WAKING_UP_MSTATE);
        }
        blinkingLEDEffect(1000, CRGB::DeepSkyBlue);
    }

    if (isMState(WAKING_UP_MSTATE)) {
        ble_uart.pollACI(); // checks for new events on the nRF8001 when wake up
        ble_state = ble_uart.getState(); // get ble next current state when wake up

        mstate = 0x00;  // reset aqs state to default
        // subscribed = false;

        pm25_lvl_alert_index = 0;
        voc_lvl_alert_index = 0;
        pm25_lvl = 0;
        ema_pm25_lvl = -1;
        voc_lvl = -1;
        ema_voc_lvl = -1;
        temp = 0;
        ema_temp = -1;
        humidity = 0;
        ema_humidity = -1;

        pm25_lvl_alert_color = CRGB::Lime;
        voc_lvl_alert_color = CRGB::Lime;

        // writeGeneralSettingToEEPROM(
        //     led_brightness_lvl,
        //     sample_count,
        //     minterval_s
        // );
        // writeSensorCalibrationSettingToEEPROM(
        //     cDatestamp,
        //     638,
        //     pm25_low_lvl_target_threshold,
        //     350000,
        //     voc_ref_co_ppm,
        //     voc_ref_co_ppm_target_window
        // );

        readSensorCalibrationSettingFromEEPROM(
            &cDatestamp,
            &pm25_low_lvl_mv,
            &pm25_low_lvl_target_threshold,
            &voc_ro,
            &voc_ref_co_ppm,
            &voc_ref_co_ppm_target_window
        );
        readGeneralSettingFromEEPROM(
            &led_brightness_lvl,
            &sample_count,
            &minterval_s
        );

        #if __RUN_CALIBRATION_WHEN_WAKING_UP__
            // pm25_low_lvl_mv = calibratePM25Sensor(
            //     CALIBRATION_MAX_CYCLE_COUNT,
            //     MAX_SAMPLE_COUNT,
            //     PM25_SENSOR_DEFAULT_LOW_LVL_TARGET_THRESHOLD
            // );

            // digitalWrite(VOC_SENSOR_EN_NOT_D, LOW);
            // delay(60000);
            // voc_ro = calibrateVOCSensor(
            //     CALIBRATION_MAX_CYCLE_COUNT,
            //     MAX_SAMPLE_COUNT,
            //     VOC_SENSOR_NATURAL_CO_PPM,
            //     VOC_SENSOR_NATURAL_CO_PPM_TARGET_WINDOW
            // );
            // digitalWrite(VOC_SENSOR_EN_NOT_D, HIGH);

            // writeSensorCalibrationSettingToEEPROM(
            //     cDatestamp,
            //     pm25_low_lvl_mv,
            //     pm25_low_lvl_target_threshold,
            //     voc_ro,
            //     voc_ref_co_ppm,
            //     voc_ref_co_ppm_target_window
            // );
        #endif

        if (ble_state == ACI_EVT_CONNECTED) {
            tx_pkg_buff_byte_count = 0;
            if (onTransmiting(PAYLOAD_EXITING_SLEEP_STATE_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                tx_pkg_buff_byte_count = 0;
            }
            setMState(LINKED_MSTATE);
        } else if (ble_state == ACI_EVT_DISCONNECTED || ble_state == ACI_EVT_DEVICE_STARTED) {
            subscribed = false;
        }

        LEDS.setBrightness(led_brightness_lvl);

        wakingUpLEDEffect(WAKE_UP_DELAY_S * 1000);

        periodic_task_runner.setInterval(minterval_s * 1000); // start sensor sampling period at every interval

        setMState(SAMPLING_MSTATE);
        setMState(MONITOR_MSTATE);
        unsetMState(WAKING_UP_MSTATE);
    }

    if (isMState(MONITOR_MSTATE)) {
        ble_uart.pollACI(); // constantly checks for new events on the nRF8001
        ble_state = ble_uart.getState(); // get ble next current state

        if (!periodic_task_runner.isRunning()) {
            periodic_task_runner.start();
        }
        periodic_task_runner.run();

        if (show_whoami_counter > 0) {
            whoamiLEDEffect(1000);
            show_whoami_counter--;
        } else {
            if (!isMState(SAMPLING_MSTATE)) {
                if (ema_pm25_lvl >= 0 && ema_pm25_lvl <= 12.1f) {
                    pm25_lvl_alert_color = CRGB::Lime;
                    pm25_lvl_alert_index = 0;
                } else if (ema_pm25_lvl > 12.1f && ema_pm25_lvl <= 35.5f) {
                    pm25_lvl_alert_color = CRGB::Yellow;
                    pm25_lvl_alert_index = 1;
                } else if (ema_pm25_lvl > 35.5f && ema_pm25_lvl <= 55.5f) {
                    pm25_lvl_alert_color = CRGB::DarkOrange;
                    pm25_lvl_alert_index = 2;
                } else if (ema_pm25_lvl > 55.5f && ema_pm25_lvl <= 150.5f) {
                    pm25_lvl_alert_color = CRGB::Red;
                    pm25_lvl_alert_index = 3;
                } else if (ema_pm25_lvl > 150.5f) {
                    pm25_lvl_alert_color = CRGB::Crimson;
                    pm25_lvl_alert_index = 4;
                }

                if (isMState(DOCKED_MSTATE)) {
                    if (ema_voc_lvl >= 0.8f) {
                        voc_lvl_alert_color = CRGB::Lime;
                        voc_lvl_alert_index = 0;
                    } else if (ema_voc_lvl < 0.8f && ema_voc_lvl >= 0.5f) {
                        voc_lvl_alert_color = CRGB::Yellow;
                        voc_lvl_alert_index = 1;
                    } else if (ema_voc_lvl < 0.5f && ema_voc_lvl >= 0.3f) {
                        voc_lvl_alert_color = CRGB::DarkOrange;
                        voc_lvl_alert_index = 2;
                    } else if (ema_voc_lvl < 0.3f && ema_voc_lvl >= 0.1f) {
                        voc_lvl_alert_color = CRGB::Red;
                        voc_lvl_alert_index = 3;
                    } else if (ema_voc_lvl < 0.1f && ema_voc_lvl >= 0) {
                        voc_lvl_alert_color = CRGB::Crimson;
                        voc_lvl_alert_index = 4;
                    }

                    if (pm25_lvl_alert_index >= voc_lvl_alert_index) {
                        monitoringLEDEffect(1000, pm25_lvl_alert_color, pm25_lvl_alert_index == 4);
                    } else {
                        monitoringLEDEffect(1000, voc_lvl_alert_color, voc_lvl_alert_index == 4);
                    }
                } else {
                    monitoringLEDEffect(1000, pm25_lvl_alert_color, pm25_lvl_alert_index == 4);
                }
            } else {
                blinkingLEDEffect(1000, CRGB::DeepSkyBlue);
            }
        }

        if (!isMState(LINKED_MSTATE) && ble_state == ACI_EVT_CONNECTED) {
            setMState(LINKED_MSTATE);
        }
        if (isMState(LINKED_MSTATE) && (ble_state == ACI_EVT_DISCONNECTED || ble_state == ACI_EVT_DEVICE_STARTED)) {
            subscribed = false;
            unsetMState(LINKED_MSTATE);
        }

        if (isDocked()) {
            if (!isMState(DOCKED_MSTATE)) {
                digitalWrite(VOC_SENSOR_EN_NOT_D, LOW);
                Serial1.begin(9600); // open dock station serial 1 port at 9600 bps

                if (docking_counter < DOCKING_DONE_COUNT) {
                    docking_counter++;
                } else {
                    setMState(DOCKED_MSTATE);

                    #if __DEBUG__
                        // Serial.println("aqs docked.");
                    #endif
                }
            }
        } else {
            if (isMState(DOCKED_MSTATE)) {
                digitalWrite(VOC_SENSOR_EN_NOT_D, HIGH);
                Serial1.end(); // close dock station serial 1 port
                docking_counter = 0;

                unsetMState(DOCKED_MSTATE);

                #if __DEBUG__
                    // Serial.println("aqs undocked.");
                #endif
            }
        }

        #if __DEBUG__
            loop_counter++;

            if (touch_hold_counter == 0) {
                if (isMState(SAMPLING_MSTATE)) {
                    Serial.print("#");
                } else {
                    Serial.print(".");
                }
            } else {
                Serial.print(touch_hold_counter);
            }
            if (loop_counter >= minterval_s) {
                loop_counter = 0;
            }
        #endif
    }

    if (isMState(SAMPLING_MSTATE)) {
        batt_lvl = readBatteryLvl();

        readTH(&temp, &humidity);
        if (sensorSampling(
            sample_count,
            pm25_low_lvl_mv,
            voc_ro,
            temp, humidity,
            &pm25_lvl,
            &voc_lvl
        )) {
            if (ema_temp >= 0) {
                ema_temp =  emaFilter(ema_temp, temp, EMA_DAMPING_FACTOR_NORMAL);
            } else {
                ema_temp = temp;
            }

            if (ema_humidity >= 0) {
                ema_humidity = emaFilter(ema_humidity, humidity, EMA_DAMPING_FACTOR_NORMAL);
            } else {
                ema_humidity = humidity;
            }

            if (ema_pm25_lvl >= 0) {
                if (abs(pm25_lvl - ema_pm25_lvl) >= (PM25_LVL_NORMAL_DAMPING_THRESHOLD)) {
                    ema_pm25_lvl = emaFilter(ema_pm25_lvl, pm25_lvl, EMA_DAMPING_FACTOR_NORMAL);
                } else {
                    ema_pm25_lvl = emaFilter(ema_pm25_lvl, pm25_lvl, EMA_DAMPING_FACTOR_HIGH);
                }
            } else {
                ema_pm25_lvl = pm25_lvl;
            }

            if (ema_voc_lvl >= 0) {
                // if (abs(voc_lvl - ema_voc_lvl) > VOC_LVL_NORMAL_DAMPING_THRESHOLD) {
                //     ema_voc_lvl = emaFilter(ema_voc_lvl, voc_lvl, EMA_DAMPING_FACTOR_NORMAL);
                // } else {
                //     ema_voc_lvl = emaFilter(ema_voc_lvl, voc_lvl, EMA_DAMPING_FACTOR_HIGH);
                // }
                ema_voc_lvl = emaFilter(ema_voc_lvl, voc_lvl, EMA_DAMPING_FACTOR_NORMAL);
            } else {
                ema_voc_lvl = voc_lvl;
            }

            #if __DEBUG__
                Serial.println();
                // Serial.print("CDate:"); Serial.println(cDatestamp);
                Serial.print("Batt: "); Serial.print(batt_lvl); Serial.println(" %");
                Serial.print("TH: "); Serial.print(ema_temp); Serial.print(" C, "); Serial.print(ema_humidity); Serial.println(" %");
                Serial.print("PM25: "); Serial.print(ema_pm25_lvl); Serial.print(" ug/m^3"); Serial.print(" @"); Serial.print(pm25_low_lvl_mv); Serial.println(" mV");
                Serial.print("VOC: "); Serial.print(ema_voc_lvl); Serial.print(" rs/ro"); Serial.print(" @") ; Serial.print(voc_ro); Serial.println(" Ohm");
                Serial.println();
            #endif

            if (subscribed && (isMState(LINKED_MSTATE) || isMState(DOCKED_MSTATE))) {
                tx_pkg_buff_byte_count = 7;
                tx_pkg_buff[0] = (uint8_t)batt_lvl;
                tx_pkg_buff[1] = (uint8_t)((uint16_t)round(temp * 100) >> 8);
                tx_pkg_buff[2] = (uint8_t)(uint16_t)round(temp * 100);

                tx_pkg_buff[3] = (uint8_t)((uint16_t)round(humidity * 100) >> 8);
                tx_pkg_buff[4] = (uint8_t)((uint16_t)round(humidity * 100));

                tx_pkg_buff[5] = (uint8_t)((uint16_t)round(ema_pm25_lvl * 100) >> 8);
                tx_pkg_buff[6] = (uint8_t)((uint16_t)round(ema_pm25_lvl * 100));

                if (isMState(DOCKED_MSTATE)) {
                    tx_pkg_buff_byte_count = 9;
                    tx_pkg_buff[7] = (uint8_t)((uint16_t)round(ema_voc_lvl * 100) >> 8);
                    tx_pkg_buff[8] = (uint8_t)((uint16_t)round(ema_voc_lvl * 100));
                }

                if (onTransmiting(PAYLOAD_SAMPLING_ENDED_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                    memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                    tx_pkg_buff_byte_count = 0;
                }
            }

            unsetMState(SAMPLING_MSTATE);
        } else {
            if (subscribed && hasMStateChanged() && (isMState(LINKED_MSTATE) || isMState(DOCKED_MSTATE))) {
                tx_pkg_buff_byte_count = 0;
                if (onTransmiting(PAYLOAD_SAMPLING_STARTED_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                    memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                    tx_pkg_buff_byte_count = 0;
                }
            }
        }
    }

    if (isMState(SENSOR_CALIBRATION_MSTATE) && (isMState(LINKED_MSTATE) || isMState(DOCKED_MSTATE))) {
        periodic_task_runner.stop();
        tx_pkg_buff_byte_count = 0;

        if (onTransmiting(PAYLOAD_CALIBRATION_STARTED_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
            memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
            tx_pkg_buff_byte_count = 0;
        }

        if (selected_sensor_for_calibration == PM25_SENSOR) {
            tx_pkg_buff_byte_count = 3;
            tx_pkg_buff[0] = (uint8_t)PM25_SENSOR;
            tx_pkg_buff[1] = (uint8_t)(pm25_low_lvl_mv >> 8);
            tx_pkg_buff[2] = (uint8_t)pm25_low_lvl_mv;

            pm25_low_lvl_mv = calibratePM25Sensor(CALIBRATION_MAX_CYCLE_COUNT, MAX_SAMPLE_COUNT, pm25_low_lvl_target_threshold);
        } else if (selected_sensor_for_calibration == VOC_SENSOR) {
            if (isMState(DOCKED_MSTATE)) {
                voc_ro = calibrateVOCSensor(CALIBRATION_MAX_CYCLE_COUNT, MAX_SAMPLE_COUNT, voc_ref_co_ppm, voc_ref_co_ppm_target_window);

                tx_pkg_buff_byte_count = 5;
                tx_pkg_buff[0] = (uint8_t)VOC_SENSOR;
                tx_pkg_buff[1] = (uint8_t)(voc_ro >> 24);
                tx_pkg_buff[2] = (uint8_t)(voc_ro >> 16);
                tx_pkg_buff[3] = (uint8_t)(voc_ro >> 8);
                tx_pkg_buff[4] = (uint8_t)voc_ro;
            }
        }

        if (onTransmiting(PAYLOAD_CALIBRATION_ENDED_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
            memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
            tx_pkg_buff_byte_count = 0;
        }

        writeSensorCalibrationSettingToEEPROM(
            cDatestamp,
            pm25_low_lvl_mv,
            pm25_low_lvl_target_threshold,
            voc_ro,
            voc_ref_co_ppm,
            voc_ref_co_ppm_target_window
        );

        unsetMState(SENSOR_CALIBRATION_MSTATE);
    }

    if (isMState(LINKED_MSTATE) || isMState(DOCKED_MSTATE)) {
        switch (onReceiving(rx_pkg_buff, &rx_pkg_buff_byte_count)) {
            case 0x00:
                break;
            case PAYLOAD_WHOAMI_HDR:
                show_whoami_counter = SHOW_WHOAMI_MAX_COUNT;
                break;
            case PAYLOAD_INFO_HDR:
                tx_pkg_buff_byte_count = strlen(AQS_REV) + strlen(AQS_SKEW) + strlen(AQS_FIRMWARE_VER) + 5;
                if (tx_pkg_buff_byte_count <= TXRX_PAYLOAD_BUFF_SIZE) {

                    tx_pkg_buff[0] = (uint8_t)(isDocked());
                    tx_pkg_buff[1] = (uint8_t)(cDatestamp >> 8);
                    tx_pkg_buff[2] = (uint8_t)(cDatestamp);
                    strcpy((char*)(tx_pkg_buff + 3), AQS_REV);
                    strcat((char*)(tx_pkg_buff + 3), ";");
                    strcat((char*)(tx_pkg_buff + 3), AQS_SKEW);
                    strcat((char*)(tx_pkg_buff + 3), ";");
                    strcat((char*)(tx_pkg_buff + 3), AQS_FIRMWARE_VER);

                    if (onTransmiting(PAYLOAD_INFO_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                        memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                        tx_pkg_buff_byte_count = 0;
                    }
                }
                break;
            case PAYLOAD_STATUS_HDR:
                tx_pkg_buff_byte_count = 1;
                if (tx_pkg_buff_byte_count <= TXRX_PAYLOAD_BUFF_SIZE) {

                    tx_pkg_buff[0] = (uint8_t)(isDocked());

                    if (onTransmiting(PAYLOAD_STATUS_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                        memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                        tx_pkg_buff_byte_count = 0;
                    }
                }
                break;
            case PAYLOAD_GENERAL_SETTING_HDR:
                tx_pkg_buff_byte_count = 3;
                if (tx_pkg_buff_byte_count <= TXRX_PAYLOAD_BUFF_SIZE) {

                    tx_pkg_buff[0] = (uint8_t)led_brightness_lvl;
                    tx_pkg_buff[1] = (uint8_t)sample_count;
                    tx_pkg_buff[2] = (uint8_t)minterval_s;

                    if (onTransmiting(PAYLOAD_GENERAL_SETTING_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                        memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                        tx_pkg_buff_byte_count = 0;
                    }
                }
                break;
            case PAYLOAD_CALIBRATION_SETTING_HDR:
                tx_pkg_buff_byte_count = 12;
                if (tx_pkg_buff_byte_count <= TXRX_PAYLOAD_BUFF_SIZE) {

                    tx_pkg_buff[0] = (uint8_t)(pm25_low_lvl_mv >> 8);
                    tx_pkg_buff[1] = (uint8_t)(pm25_low_lvl_mv);

                    tx_pkg_buff[2] = (uint8_t)((uint16_t)(pm25_low_lvl_target_threshold * 100) >> 8);
                    tx_pkg_buff[3] = (uint8_t)((uint16_t)(pm25_low_lvl_target_threshold * 100));

                    tx_pkg_buff[4] = (uint8_t)(voc_ro >> 24);
                    tx_pkg_buff[5] = (uint8_t)(voc_ro >> 16);
                    tx_pkg_buff[6] = (uint8_t)(voc_ro >> 8);
                    tx_pkg_buff[7] = (uint8_t)(voc_ro);

                    tx_pkg_buff[8] = (uint8_t)((uint16_t)(voc_ref_co_ppm * 100) >> 8);
                    tx_pkg_buff[9] = (uint8_t)((uint16_t)(voc_ref_co_ppm * 100));

                    tx_pkg_buff[10] = (uint8_t)((uint16_t)(voc_ref_co_ppm_target_window * 100) >> 8);
                    tx_pkg_buff[11] = (uint8_t)((uint16_t)(voc_ref_co_ppm_target_window * 100));

                    if (onTransmiting(PAYLOAD_CALIBRATION_SETTING_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                        memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                        tx_pkg_buff_byte_count = 0;
                    }
                }
                break;
            case PAYLOAD_UPDATE_GENERAL_SETTING_HDR:
                if (rx_pkg_buff_byte_count == 3) {
                    led_brightness_lvl = rx_pkg_buff[0];

                    sample_count = rx_pkg_buff[1];
                    if (sample_count < MIN_SAMPLE_COUNT) {
                        sample_count = MIN_SAMPLE_COUNT;
                        merror_code = SAMPLE_COUNT_MERROR;
                    } else if (sample_count > MAX_SAMPLE_COUNT) {
                        sample_count = MAX_SAMPLE_COUNT;
                        merror_code = SAMPLE_COUNT_MERROR;
                    }

                    minterval_s = rx_pkg_buff[2];
                    if (minterval_s < sample_count + MIN_MONITOR_IDLE_COUNT) {
                        minterval_s = sample_count + MIN_MONITOR_IDLE_COUNT;
                        merror_code = MONITOR_INTERVAL_MERROR;
                    } else if (minterval_s < MAX_MONITORING_INTERVAL_S) {
                        minterval_s = MAX_MONITORING_INTERVAL_S;
                        merror_code = MONITOR_INTERVAL_MERROR;
                    }

                    periodic_task_runner.setInterval(minterval_s * 1000); // aqs periodic monitor interval (ms)

                    LEDS.setBrightness(led_brightness_lvl);

                    writeGeneralSettingToEEPROM(
                        led_brightness_lvl,
                        sample_count,
                        minterval_s
                    );
                    setMState(MONITOR_MSTATE);
                } else if (rx_pkg_buff_byte_count > TXRX_PAYLOAD_BUFF_SIZE) {
                    merror_code = RX_PAYLOAD_BUFF_OVERFLOW_MERROR;
                } else {
                    merror_code = INCORRECT_PAYLOAD_SIZE_MERROR;
                }
                break;
            case PAYLOAD_UPDATE_CALIBRATION_SETTING_HDR:
                if (rx_pkg_buff_byte_count == 5 || rx_pkg_buff_byte_count == 9) {
                    selected_sensor_for_calibration = rx_pkg_buff[0];
                    if (selected_sensor_for_calibration == PM25_SENSOR) {
                        pm25_low_lvl_mv = (uint16_t)(rx_pkg_buff[1] << 8) + (uint16_t)rx_pkg_buff[2];
                        pm25_low_lvl_target_threshold = (float)(
                            ((uint16_t)rx_pkg_buff[3] << 8) +
                            (uint16_t)rx_pkg_buff[4]
                        ) / 100;
                    } else if (selected_sensor_for_calibration == VOC_SENSOR) {
                        voc_ro = ((uint32_t)rx_pkg_buff[1] << 24) +
                            ((uint32_t)rx_pkg_buff[2] << 16) +
                            ((uint32_t)rx_pkg_buff[3] << 8) +
                            (uint32_t)rx_pkg_buff[4];
                        voc_ref_co_ppm = (float)(
                            ((uint16_t)rx_pkg_buff[5] << 8) +
                            (uint16_t)rx_pkg_buff[6]
                        ) / 100;
                        voc_ref_co_ppm_target_window = (float)(
                            ((uint16_t)rx_pkg_buff[7] << 8) +
                            (uint16_t)rx_pkg_buff[8]
                        ) / 100;
                    }

                    writeSensorCalibrationSettingToEEPROM(
                        cDatestamp,
                        pm25_low_lvl_mv,
                        pm25_low_lvl_target_threshold,
                        voc_ro,
                        voc_ref_co_ppm,
                        voc_ref_co_ppm_target_window
                    );
                } else if (rx_pkg_buff_byte_count > TXRX_PAYLOAD_BUFF_SIZE) {
                    merror_code = RX_PAYLOAD_BUFF_OVERFLOW_MERROR;
                } else {
                    merror_code = INCORRECT_PAYLOAD_SIZE_MERROR;
                }
                break;
            case PAYLOAD_START_CALIBRATION_HDR:
                if (rx_pkg_buff_byte_count == 4) {
                    selected_sensor_for_calibration = rx_pkg_buff[0];
                    cDatestamp = ((uint16_t)rx_pkg_buff[1] << 8) + (uint16_t)rx_pkg_buff[2];

                    setMState(SENSOR_CALIBRATION_MSTATE);
                } else if (rx_pkg_buff_byte_count > TXRX_PAYLOAD_BUFF_SIZE) {
                    merror_code = RX_PAYLOAD_BUFF_OVERFLOW_MERROR;
                } else {
                    merror_code = INCORRECT_PAYLOAD_SIZE_MERROR;
                }
                break;
            case PAYLOAD_START_SUBSCRIPTION_HDR:
                subscribed = true;
                tx_pkg_buff_byte_count = 0;
                if (onTransmiting(PAYLOAD_SUBSCRIPTION_STARTED_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                    memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                    tx_pkg_buff_byte_count = 0;
                }
                break;
            case PAYLOAD_STOP_SUBSCRIPTION_HDR:
                subscribed = false;
                tx_pkg_buff_byte_count = 0;
                if (onTransmiting(PAYLOAD_SUBSCRIPTION_ENDED_HDR, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                    memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                    tx_pkg_buff_byte_count = 0;
                }
                break;
            default:
                merror_code = UNKNOW_PAYLOAD_HDR_MERROR;
                break;
        }

        if (merror_code != 0x00) {
            tx_pkg_buff_byte_count = 0;
            if (onTransmiting(PAYLOAD_MERROR_CODE_HDR | merror_code, tx_pkg_buff, tx_pkg_buff_byte_count)) {
                memset(tx_pkg_buff, 0, sizeof(tx_pkg_buff));
                tx_pkg_buff_byte_count = 0;
            }
            merror_code = 0x00;
        }

        memset(rx_pkg_buff, 0, sizeof(rx_pkg_buff));
        rx_pkg_buff_byte_count = 0;
    }
    prev_mstate = mstate;
}
/*------------------------------------------------------------------------
 * DESCRIPTION - AQS setup. Configure the Arduino and startup BLE, DHT sensor, and PM2.5 sensor.
 *
 * FUNCTION: setup
 ------------------------------------------------------------------------*/
void setup (void) {
    #if __DEBUG__
        Serial.begin(9600); // open debug serial 1 port at 9600 bps

        delay(10000);

        Serial.println("Virida");
        Serial.print("FW: "); Serial.println(AQS_FIRMWARE_VER);
        Serial.print("Rev: "); Serial.print(AQS_REV); Serial.println(AQS_SKEW);
        Serial.println();
    #endif

    /* setup the MICS5524 VOC sensor and Sharp GP2Y1010AU0F PM2.5 sensor pins */
    pinMode(VOC_SENSOR_EN_NOT_D, OUTPUT);
    pinMode(PM25_SENSOR_EN_NOT_D, OUTPUT);
    digitalWrite(VOC_SENSOR_EN_NOT_D, HIGH); // VOC sensor enable not default to high
    digitalWrite(PM25_SENSOR_EN_NOT_D, HIGH); // PM2.5 sensor ILED default to high

    /* setup touch interupt pin */
    pinMode(TOUCH_INTRPT_D, INPUT);

    /* setup docking detect pin */
    pinMode(DOCKING_DET_NOT_D, INPUT_PULLUP);
    digitalWrite(DOCKING_DET_NOT_D, HIGH);

    /* setup aqs leds */
    LEDS.addLeds<WS2812B, MLEDS_D, GRB>(mleds, MLED_COUNT).setCorrection(TypicalSMD5050);
    LEDS.setMaxPowerInMilliWatts(MAX_LED_PWR_CONSUMPTION_MW);

    periodic_task_runner.expiredHandler(periodicTaskHandler);

    ble_uart.setDeviceName(AQS_BLE_DEVICE_ALIAS);
    ble_uart.begin();

    setMState(WAKING_UP_MSTATE);
}
/*------------------------------------------------------------------------
 * DESCRIPTION - AQS app run loop.
 *
 * FUNCTION: loop
 ------------------------------------------------------------------------*/
void loop (void) {
    runApp();
}
