//
// Created by Cameloah on 29/10/2022.
//

#pragma once

#include "EEPROM.h"


#define SYS_CONTROL_SAVE_MILEAGE

#define EEPROM_SIZE                     128
#define GPS_SERIAL_BAUD_RATE            9600

#define PIN_RX                          16
#define PIN_TX                          17

#define GPS_INIT_TIMEOUT                5000

#define INTERVAL_DISTANCE_M             200
#define INTERVAL_GPS_MEASURE_MS         5000

typedef enum{
    GPS_MANAGER_ERROR_NO_ERROR          = 0x00,
    GPS_MANAGER_ERROR_LOCATION          = 0x01,
    GPS_MANAGER_ERROR_SATS              = 0x02,
    GPS_MANAGER_ERROR_TIMEOUT           = 0x03,
    GPS_MANAGER_ERROR_UNKNOWN           = 0xFF
} GPS_MANAGER_ERROR_t;

struct GpsDataState_t {
    double posLat = 0;
    double posLon = 0;
    double prevPosLat = 0;
    double prevPosLon = 0;
    double mileage_km = 0;
    uint8_t numberSats = 0;
};

extern GpsDataState_t gpsState;

/// \brief Initializes gps module and waits for initial data for a predefined time
///
/// \param timeout pass a timout in ms
/// \return GPS_MANAGER_ERROR_t
GPS_MANAGER_ERROR_t gps_module_init(unsigned long timeout);

/// \brief Sets up and initializes the serial communication with the gps module GY-NEO6MV2
///
/// \return none
void gps_manager_init();

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

/// \brief returns initialization status of gps module
///
/// \return bool, true if initialized
bool gps_manager_is_init();