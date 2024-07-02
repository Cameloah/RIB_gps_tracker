//
// Created by Cameloah on 29/10/2022.
//

#include "TinyGPSPlus.h"
#include "HardwareSerial.h"
#include "gps_manager.h"
#include "tools/loop_timer.h"
#include "ram_log.h"
#include "main_project_utils.h"
#include "memory_module.h"


MemoryModule gps_parameters;


TinyGPSPlus gps_obj;
HardwareSerial SerialGPS(1);                             // gps connected to UART 1
bool flag_initialized = false;

uint32_t counter_gps_update = 0;                                // counter of loopiterations without checking gps pos

GpsDataState_t gpsState = {};

// custom version of delay() ensures that the gps object is being updated.
static void _smart_delay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (SerialGPS.available())
            gps_obj.encode((char) SerialGPS.read());
    } while (millis() - start < ms);
}

GPS_MANAGER_ERROR_t _update_pos(unsigned long timeout, uint8_t sats) {
    unsigned long start = millis();
    do {
        // pull data
        while (SerialGPS.available())
            gps_obj.encode((char) SerialGPS.read());

        // check for satellites
        if (gps_obj.satellites.isValid() && sats <= gps_obj.satellites.value()) {
            // notify ramlog
            if (sats > gpsState.numberSats) {
                String str_log =
                        "Satelliten-Genauigkeit hergestellt. Satelliten: " + String(gps_obj.satellites.value());
                ram_log_notify(RAM_LOG_INFO, str_log.c_str(), true);
            }

            // update number of satellites
            gpsState.numberSats = gps_obj.satellites.value();

            // check for validity
            if (gps_obj.location.isValid() && gps_obj.location.isUpdated()) {
                // measurement is valid and fresh, therefore save position
                gpsState.posLat = gps_obj.location.lat();
                gpsState.posLon = gps_obj.location.lng();
                // all done, so return
                return GPS_MANAGER_ERROR_NO_ERROR;
            }
        }
    } while (millis() - start < timeout);
    // we timed out. the position measurement is not valid
    if (gpsState.numberSats < sats)
        return GPS_MANAGER_ERROR_SATS;
    return GPS_MANAGER_ERROR_LOCATION;
}

bool check_within_circle(double x, double y, double x_center, double y_center, double radius) {
    double distance = sqrt(pow(x - x_center, 2) + pow(y - y_center, 2));
    return distance <= radius;
}

void gps_manager_init() {
    GPS_MANAGER_ERROR_t retval;
    memset(&gpsState, 0, sizeof(GpsDataState_t));

    pinMode(PIN_ALARM, OUTPUT);

    gps_parameters.addParameter("mileage_km", (double) 0);
    gps_parameters.loadAllStrict();


#ifdef SYS_CONTROL_SAVE_MILEAGE
    // read mileage from flash
    gpsState.mileage_km = (double) *gps_parameters.getDouble("mileage_km");
#endif

    // initialize gps module
    DualSerial.println("Initialisiere GPS-Modul...");
    if ((retval = gps_module_init(GPS_INIT_TIMEOUT)) == GPS_MANAGER_ERROR_NO_ERROR)
        flag_initialized = true;

    else {
        DualSerial.println("Fehler. GPS-Modul antwortet nicht. Neuversuch laeuft im Hintergrund.");
        ram_log_notify(RAM_LOG_ERROR_GPS_MANAGER, retval);
    }
}

GPS_MANAGER_ERROR_t gps_module_init(unsigned long timeout) {
    // init serial communication with gps module
    SerialGPS.begin(GPS_SERIAL_BAUD_RATE, SERIAL_8N1, PIN_RX, PIN_TX);

    unsigned long start = millis();
    do {
        if (SerialGPS.available()) {
            ram_log_notify(RAM_LOG_INFO, "GPS online.", true);
            flag_initialized = true;
            return GPS_MANAGER_ERROR_NO_ERROR;
        }
        delay(100);
    } while (millis() - start < timeout);

    return GPS_MANAGER_ERROR_TIMEOUT;
}

void gps_manager_update() {
    // if not initialized, try again
    if (!flag_initialized)
        if (gps_module_init(GPS_INIT_TIMEOUT) != GPS_MANAGER_ERROR_NO_ERROR)
            return;

    // we always pull data from the gps module
    _smart_delay(0);

    // but we only check the current position periodically
    counter_gps_update++;
    if (counter_gps_update * 1000 / FREQ_LOOP_CYCLE_HZ > INTERVAL_GPS_MEASURE_MS) {
        counter_gps_update = 0;

        GPS_MANAGER_ERROR_t retval;
        // read gps coordinates from module
        if ((retval = _update_pos(5000, 4)) == GPS_MANAGER_ERROR_NO_ERROR) {
            // we have a valid measurement so lets check whether we have a previous measurement
            if (gpsState.prevPosLat == 0 || gpsState.prevPosLon == 0) {
                // we don't have prev measurements yet so lets save them
                gpsState.prevPosLat = gpsState.posLat;
                gpsState.prevPosLon = gpsState.posLon;
                // and return early
                return;
            }

            // calculate route kilometers
            double interval_m = TinyGPSPlus::distanceBetween(gpsState.posLat, gpsState.posLon, gpsState.prevPosLat,
                                                             gpsState.prevPosLon);

            gpsState.speed_kmh = gps_obj.speed.kmph();


            if (interval_m > INTERVAL_DISTANCE_M) {
                // we have passed a distance of x meters
                // update previous position
                gpsState.prevPosLat = gpsState.posLat;
                gpsState.prevPosLon = gpsState.posLon;

                // save to milage
                gpsState.mileage_km += interval_m / 1000.0;

#ifdef SYS_CONTROL_SAVE_MILEAGE
                gps_parameters.set("mileage_km", gpsState.mileage_km, true);
#endif
            }
        } else if (retval == GPS_MANAGER_ERROR_SATS) {
            String str_log = "Satelliten-Genauigkeit verloren. Satelliten: " + String(gpsState.numberSats);
            ram_log_notify(RAM_LOG_INFO, str_log.c_str());
        }
    }

    if (gpsState.speed_kmh > SPEED_LIMIT_HABOUR && 
        check_within_circle(gpsState.posLat, gpsState.posLon,
                            LAT_ANKLAM_CENTER, LON_ANKLAM_CENTER,
                            RADIUS_ANKLAM_CENTER)) {
        if(millis() / 1000 % 2 == 0) {
            digitalWrite(PIN_ALARM, LOW);
            }
        else
            digitalWrite(PIN_ALARM, HIGH);
    }

    else
        digitalWrite(PIN_ALARM, HIGH);
}

bool gps_manager_is_init() {
    return flag_initialized;
}