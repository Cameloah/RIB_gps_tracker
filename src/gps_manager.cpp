//
// Created by Jo Uni on 29/10/2022.
//

#include "TinyGPSPlus.h"
#include "HardwareSerial.h"
#include "gps_manager.h"
#include "tools/loop_timer.h"

TinyGPSPlus gps_obj;
HardwareSerial SerialGPS(1);

uint32_t counter_serial_update = 0;

GpsDataState_t gpsState = {};

void gps_manager_init() {
    // init serial communication with gps module
    SerialGPS.begin(GPS_SERIAL_BAUD_RATE, SERIAL_8N1, PIN_RX, PIN_TX);

    // read origin position from flash
    long readValue;
    EEPROM_readAnything(0, readValue);
    gpsState.originLat = (double)readValue / 1000000;

    EEPROM_readAnything(4, readValue);
    gpsState.originLon = (double)readValue / 1000000;

    EEPROM_readAnything(8, readValue);
    gpsState.originAlt = (double)readValue / 1000000;

    // read milage from flash
    EEPROM_readAnything(12, readValue);
    gpsState.milage_km = (double)readValue / 1000000;

    // wait for gps module
    Serial.println("Warte auf erste GPS daten...");
    while (!SerialGPS.available())
        delay(100);

    Serial.println("GPS online. Warte auf Satelliten...");
    do {
        while (SerialGPS.available()) {
            gps_obj.encode(SerialGPS.read());
        }

        Serial.print("Anzahl Satelliten: ");
        Serial.println(gps_obj.satellites.value());
    } while (gps_obj.satellites.value() < 5);

    Serial.print(gps_obj.satellites.value());  Serial.println(" Satelliten gefunden.");

    // read gps coordinates from module
    gpsState.posLat = gps_obj.location.lat();
    gpsState.posLon = gps_obj.location.lng();
    gpsState.posAlt = gps_obj.altitude.meters();

    // save home position
#ifdef SYS_CONTROL_SET_HOME_POS
    Serial.println("Setze Heimat-Position.");
    long writeValue = gpsState.posLat * 1000000;
    EEPROM_writeAnything(0, writeValue);

    writeValue = gpsState.posLon * 1000000;
    EEPROM_writeAnything(4, writeValue);

    writeValue = gpsState.posAlt * 1000000;
    EEPROM_writeAnything(8, writeValue);
    EEPROM.commit(); // commit data to flash
#endif

    // and reset previous position
    gpsState.prevPosLat = gpsState.posLat;
    gpsState.prevPosLon = gpsState.posLon;
    gpsState.prevPosAlt = gpsState.posAlt;
}

void gps_manager_update() {

    while (SerialGPS.available() > 0) {
        gps_obj.encode(SerialGPS.read());
    }

    // read gps coordinates from module
    gpsState.posLat = gps_obj.location.lat();
    gpsState.posLon = gps_obj.location.lng();
    gpsState.posAlt = gps_obj.altitude.meters();

    /*
     * Diverse Berechnungen von Maximum und Minimum-Werten und zurückgelegter Distanz
     * Diese werden aber erst gemacht, wenn mindestens ein Fix mit 4 Satelliten vorhanden
     * ist, allenfalls wäre die Genauigkeit nicht gegeben und es würden falsche
     * Werte berechnet werden.
     */
    if (gps_obj.satellites.value() > 4) {

        // calculate direct distance to home
        gpsState.distToOrigin = TinyGPSPlus::distanceBetween(gpsState.posLat, gpsState.posLon, gpsState.originLat, gpsState.originLon);

        // keep track of maximum direct distance to home
        if (gpsState.distToOrigin > gpsState.distToOriginMax) {
            gpsState.distToOriginMax = gpsState.distToOrigin;
        }

        // keep track of speed
        if (gps_obj.speed.kmph() > gpsState.spdMax_kmph) {
            gpsState.spdMax_kmph = gps_obj.speed.kmph();
        }

        // record route kilometers
        double interval_m = TinyGPSPlus::distanceBetween(gpsState.posLat, gpsState.posLon, gpsState.prevPosLat, gpsState.prevPosLon);
        if (interval_m > INTERVAL_M && interval_m < 10000) {
            // we have passed a distance of x meters therefore save to milage
            gpsState.milage_km += interval_m / 1000.0;
            // now save to eeprom
            long writeValue = gpsState.milage_km * 1000000;
            EEPROM_writeAnything(12, writeValue);
            EEPROM.commit(); // commit data to flash

            // and reset previous position
            gpsState.prevPosLat = gpsState.posLat;
            gpsState.prevPosLon = gpsState.posLon;
            gpsState.prevPosAlt = gpsState.posAlt;

#ifdef SYS_CONTROL_SET_MILAGE
            long writeValue = SYS_CONTROL_SET_MILAGE * 1000000;
            EEPROM_writeAnything(12, writeValue);
            EEPROM.commit(); // commit data to flash

            Serial.print("Speichere neuen Kilometerstand...");
#endif
        }
    }

    /*
       Damit nicht zu viele Daten im Serial Monitor ausgegeben werden,
       beschränken wir die Ausgabe auf die Anzahl Millisekunden
       die wir im Makro "INTERVAL_SERIAL_GPS_OUTPUT_MS" gespeichert haben
    */
    counter_serial_update++;
    if (counter_serial_update * 1000/FREQ_LOOP_CYCLE_HZ > INTERVAL_SERIAL_GPS_OUTPUT_MS) {
        Serial.print("Lattitude             = ");  Serial.println(gps_obj.location.lat(), 6);
        Serial.print("Longitude             = ");  Serial.println(gps_obj.location.lng(), 6);
        Serial.print("Hoehe                 = ");  Serial.println(gps_obj.altitude.meters());
        Serial.print("Anzahl Satelliten     = ");  Serial.println(gps_obj.satellites.value());
        Serial.print("Geschwindigkeit       = ");  Serial.println(gps_obj.speed.kmph());
        Serial.print("Geschwindigkeit max   = ");  Serial.println(gpsState.spdMax_kmph);
        Serial.print("Luftlinie zu Hafen    = ");  Serial.println(gpsState.distToOrigin, 1);
        Serial.print("Luftlinie zu Hafen max= ");  Serial.println(gpsState.distToOriginMax, 1);
        Serial.print("Kilometerstand        = ");  Serial.println(gpsState.milage_km);
        counter_serial_update = 0;
    }
}