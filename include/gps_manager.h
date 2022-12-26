//
// Created by Jo Uni on 29/10/2022.
//

#pragma once

#include "EEPROM.h"


// #define SYS_CONTROL_SET_HOME_POS
// #define SYS_CONTROL_SAVE_MILAGE
// #define SYS_CONTROL_VERBOSE

#define EEPROM_SIZE                     128
#define GPS_SERIAL_BAUD_RATE            9600

#define PIN_RX                          16
#define PIN_TX                          17

#define INTERVAL_DISTANCE_M             200
#define INVERVAL_GPS_MEASURE_MS         5000
#define INTERVAL_SERIAL_GPS_OUTPUT_MS   1000
#define TH_MILAGE_SPEED_MAX             200.0  // km/h

typedef enum{
    GPS_MANAGER_ERROR_NO_ERROR          = 0x00,
    GPS_MANAGER_ERROR_TIMEOUT           = 0x01,
    GPS_MANAGER_ERROR_UNKNOWN           = 0xFF
} GPS_MANAGER_ERROR_t;

struct GpsDataState_t {
    double originLat = 0;
    double originLon = 0;
    double originAlt = 0;
    double posLat = 0;
    double posLon = 0;
    double posAlt = 0;
    double prevPosLat = 0;
    double prevPosLon = 0;
    double prevPosAlt = 0;
    double distToOrigin = 0;
    double distToOriginMax = 0;
    double spd_kmph = 0;
    double spdMax_kmph = 0;
    double milage_km = 0;
};

extern GpsDataState_t gpsState;

/// \brief Sets up and initializes the serial communication with the gps module GY-NEO6MV2
///
/// \return none
GPS_MANAGER_ERROR_t gps_manager_init();

/// \brief runs update routine for the gps module
///
/// \return none
void gps_manager_update();

/// \brief write number to eeprom
///
/// \tparam T data type to be saved
/// \param ee address where to save
/// \param value number to be saved
/// \return number of bytes saved
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return i;
}

/// \brief read data from eeprom
///
/// \tparam T data type to be returned
/// \param ee address where to read from
/// \param value variable where to save data
/// \return number of bytes read
template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}