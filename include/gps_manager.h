//
// Created by Cameloah on 29/10/2022.
//

#pragma once

#include "memory_module.h"


#define SYS_CONTROL_SAVE_MILEAGE

#define GPS_SERIAL_BAUD_RATE            9600

#define PIN_RX                          16
#define PIN_TX                          17
#define PIN_ALARM                       19

#define GPS_INIT_TIMEOUT                5000

#define INTERVAL_DISTANCE_M             200
#define INTERVAL_GPS_MEASURE_MS         2000

#define SPEED_LIMIT_HABOUR              8
#define LAT_ANKLAM_CENTER               53.858735
#define LON_ANKLAM_CENTER               13.685272
#define RADIUS_ANKLAM_CENTER            0.011953

#define LAT_ANKLAM_WEST                 53.856855
#define LON_ANKLAM_WEST                 13.675473
#define LAT_ANKLAM_EAST                 53.862664
#define LON_ANKLAM_EAST                 13.697274

#define LAT_SEESPORTCLUB_WEST           53.859465
#define LON_SEESPORTCLUB_WEST           13.717133
#define LAT_SEESPORTCLUB_EAST           53.859870
#define LON_SEESPORTCLUB_EAST           13.719998

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
    float speed_kmh = 0;
    uint8_t numberSats = 0;
};

extern GpsDataState_t gpsState;
extern MemoryModule gps_parameters;

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

/// \brief returns initialization status of gps module
///
/// \return bool, true if initialized
bool gps_manager_is_init();