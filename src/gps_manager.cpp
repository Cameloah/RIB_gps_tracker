//
// Created by Jo Uni on 29/10/2022.
//

#include "TinyGPSPlus.h"
#include "HardwareSerial.h"
#include "gps_manager.h"
#include "tools/loop_timer.h"
#include "wifi_handler.h"
#include "ram_log.h"

TinyGPSPlus gps_obj;
HardwareSerial SerialGPS(1);

uint32_t counter_serial_update = 0;
uint32_t counter_gps_update = 0;
long time_pos_update = 0;

GpsDataState_t gpsState = {};

// This custom version of delay() ensures that the gps object
// is being "fed".
static void _smart_delay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (SerialGPS.available())
            gps_obj.encode(SerialGPS.read());
    } while (millis() - start < ms);
}

GPS_MANAGER_ERROR_t _update_pos(unsigned long timeout) {
    unsigned long start = millis();
    do
    {
        // pull data
        while (SerialGPS.available())
            gps_obj.encode(SerialGPS.read());

        // check for validity
        if (gps_obj.location.isValid() && gps_obj.location.isUpdated()) {
            // measurement is valid and fresh, therefore save position
            gpsState.posLat = gps_obj.location.lat();
            gpsState.posLon = gps_obj.location.lng();
            // all done, so return out of loop
            return GPS_MANAGER_ERROR_NO_ERROR;
        }
    } while (millis() - start < timeout);
    // we timeouted. the position measurement is not valid
    return GPS_MANAGER_ERROR_TIMEOUT;
}

GPS_MANAGER_ERROR_t gps_manager_init() {
    GPS_MANAGER_ERROR_t retval;
    // init serial communication with gps module
    SerialGPS.begin(GPS_SERIAL_BAUD_RATE, SERIAL_8N1, PIN_RX, PIN_TX);

#ifdef SYS_CONTROL_SAVE_MILAGE
    // read milage from flash
    long readValue;
    EEPROM_readAnything(12, readValue);
    gpsState.milage_km = (double)readValue / 1000000;
#endif

    // wait for gps module
    DualSerial.println("Warte auf erste GPS daten...");
    while (!SerialGPS.available())
        delay(100);

    DualSerial.println("GPS online.");
    uint8_t number_sats;
    do {
        // read serial data
        _smart_delay(0);
        number_sats = 0;

        if(gps_obj.satellites.isValid() && gps_obj.satellites.isUpdated()) {
            DualSerial.print("Warte auf 4 Satelliten... Anzahl Satelliten: ");
            number_sats = gps_obj.satellites.value();
            DualSerial.println(number_sats);
        }
    } while (number_sats < 4);

    DualSerial.print(" Satelliten gefunden. Setze erste Position:");
    // lets try to measure position for 10s max duration
    if ((retval = _update_pos(10000)) != GPS_MANAGER_ERROR_NO_ERROR)
        return retval;

    DualSerial.print(gpsState.posLat); DualSerial.print(", "); DualSerial.println(gpsState.posLon);
    // and reset previous position
    gpsState.prevPosLat = gpsState.posLat;
    gpsState.prevPosLon = gpsState.posLon;

    return GPS_MANAGER_ERROR_NO_ERROR;
}

void gps_manager_update() {

    // we always pull data from the gps module
    _smart_delay(0);

    // but we only check the current position periodically
    counter_gps_update++;
    if (counter_gps_update * 1000/FREQ_LOOP_CYCLE_HZ > INVERVAL_GPS_MEASURE_MS) {
        counter_gps_update = 0;

        // read gps coordinates from module
        if (_update_pos(500) == GPS_MANAGER_ERROR_NO_ERROR) {
            // we have a valid measurement so lets caluculate route kilometers
            DualSerial.print("Berechne distanz zwischen ");
            DualSerial.print(gpsState.posLat); DualSerial.print(", "); DualSerial.print(gpsState.posLon);
            DualSerial.print(" und ");
            DualSerial.print(gpsState.prevPosLat); DualSerial.print(", "); DualSerial.println(gpsState.prevPosLon);
            double interval_m = TinyGPSPlus::distanceBetween(gpsState.posLat, gpsState.posLon, gpsState.prevPosLat,
                                                             gpsState.prevPosLon);

            if (interval_m > INTERVAL_DISTANCE_M) {
                // we have passed a distance of x meters
                // update previous position
                gpsState.prevPosLat = gpsState.posLat;
                gpsState.prevPosLon = gpsState.posLon;
                gpsState.prevPosAlt = gpsState.posAlt;

                DualSerial.print(gpsState.posLat); DualSerial.print(", "); DualSerial.println(gpsState.posLon);

                // save to milage
                gpsState.milage_km += interval_m / 1000.0;
                DualSerial.print(interval_m); DualSerial.print("m, Neuer KM-stand: "); DualSerial.println(gpsState.milage_km);

#ifdef SYS_CONTROL_SAVE_MILAGE
                long writeValue = gpsState.milage_km * 1000000;
                EEPROM_writeAnything(12, writeValue);
                EEPROM.commit(); // commit data to flash
#endif
            }
        }
    }
}