#include <Arduino.h>
#include <cstdio>
#include "wifi_debugger.h"
#include "WebServer.h"
#include "gps_manager.h"
#include "version.h"
#include "tools/loop_timer.h"

/* Changelog:
 * - 1.0.0 basic readout adapted from adafruit mpu6050 example
 *      display data and interface via serial comm
 *      calculates rot matrices from sensor to device housing and from device to ship during calibration
*/

// debug and system control options
#define SYSCTRL_LOOPTIMER               // enable loop frequency control, remember to also set the loop freq in the loop_timer.h
#define INTERVAL_WIFI_CHECK_MS          60000

WebServer server(80);
String ip_default = "192.168.2.91";

void handleRoot()
{server.send(200, "text/html",String(gpsState.milage_km));}


void setup() {
    delay(1000);
    // Setup serial communication
    Serial.begin(115200);

    // init eeprom flash
    Serial.println("Initialisiere Speichermodul...");
    while (!EEPROM.begin(EEPROM_SIZE)) {
        continue;
    }
    Serial.println("Erfolgreich.");


    // wifi setup
    uint8_t retval = WIFI_DEBUGGER_ERROR_UNKNOWN;
    Serial.println("Starte Wifi.");

    // read ip from flash
    uint8_t readValue_ip[4];
    for (int i = 0; i < 4; ++i) {
        EEPROM_readAnything(16+i, readValue_ip[i]);
    }

    if (readValue_ip[0] == 192) {
        char ip_loaded[15];
        sprintf( ip_loaded, "%d.%d.%d.%d", readValue_ip[0], readValue_ip[1], readValue_ip[2], readValue_ip[3]);
        strcpy((char*)ip_default.c_str(), ip_loaded);
        Serial.print("IP-Adresse aus Speicher geladen: ");
    }
    else Serial.print("Verwende Standard-IP: ");

    Serial.println(ip_default);

    // start wifi
    retval = wifi_debugger_init("WLAN", "kanustation", ip_default.c_str(), "192.168.2.1", "255.255.255.0", URL_FW_VERSION, URL_FW_BIN);
    if(retval == WIFI_DEBUGGER_ERROR_NO_ERROR) {
        Serial.println("Suche nach Updates...");
        retval = wifi_debugger_fwVersionCheck(FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
        if (retval == WIFI_DEBUGGER_ERROR_NO_ERROR)
            wifi_debugger_firmwareUpdate();
        else if (retval == WIFI_DEBUGGER_ERROR_NO_UPDATE)
            Serial.println("FW ist aktuell!");
        else Serial.println("Fehler.");

        // since we have wifi, lets start the server
        Serial.println("Starte Server");
        server.on("/", handleRoot);
        server.begin();
    }
    else if (retval == WIFI_DEBUGGER_ERROR_WIFI)
        Serial.println("WLAN nicht gefunden.");
    else Serial.println("Fehler.");

    // gps setup
    gps_manager_init();
}


double counter_wifi = 0;

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    counter_wifi++;
    if (!wifi_debugger_is_connected() && (counter_wifi * 1000/FREQ_LOOP_CYCLE_HZ  > INTERVAL_WIFI_CHECK_MS)) {
        counter_wifi = 0;

        // try to reconnect
        uint8_t retval = connect_wifi();
        if (retval == WIFI_DEBUGGER_ERROR_NO_ERROR) {
            // setup root callback to send data
            Serial.println("WiFi wieder verbunden. Starte Server.");
            server.on("/", handleRoot);
            server.begin();
        }
        if (retval == WIFI_DEBUGGER_ERROR_WIFI) {
            Serial.println("WLAN nicht gefunden.");
            delay(10000);

        }
        else Serial.println("Fehler.");
    }

    // wifi is connected so run server
    else server.handleClient();


    // always request gps and count milage
    gps_manager_update();

    // handle serial commands
    // listen for user input
    if (Serial.available())
        delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = Serial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        Serial.readBytes(rx_user_input, rx_available_bytes);

        // extract first word as command key
        char* rx_command_key = strtok(rx_user_input, " \n");

        // catch exception where no token was sent
        if (rx_command_key == nullptr)
            return;

        if (!strcmp(rx_command_key, "konfiguriere")) {
            char *sub_key = strtok(nullptr, " \n");

            if (sub_key == nullptr) {
                Serial.println("\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:");
                Serial.println("--ip [ip-adresse]      - ändern der gespeicherten IP-Adresse");
                Serial.println("--km [wert]            - ändern des gespeicherten km-Standes'\n");
                return;
            }

            if (!strcmp(sub_key, "--ip")) {

                uint8_t writevalue_ip[4];

                for (int i = 0; i < 4; ++i) {
                    char *sub_key = strtok(nullptr, ". \n");
                    if (sub_key == nullptr) {
                        Serial.println("Fehler.");
                        return;
                    }
                    Serial.print(writevalue_ip[i] = atoi(sub_key));
                    Serial.println(" ");
                }

                for (int i = 0; i < 4; ++i) {
                    EEPROM_writeAnything(16+i, writevalue_ip[i]);
                }
                EEPROM.commit();
                Serial.println("Speichere IP...");
                Serial.println("Starte neu...");
                delay(1000);
                esp_restart();
            }

            else if (!strcmp(sub_key, "--km")) {
                char *sub_key = strtok(nullptr, " \n");
                gpsState.milage_km = atof(sub_key);
                long writeValue = gpsState.milage_km * 1000000;
                EEPROM_writeAnything(12, writeValue);
                EEPROM.commit(); // commit data to flash

                Serial.println("Speichere neuen Kilometerstand...");
            }

            else {
                // unknown command
                Serial.println("\nUnbekannter Befehl.");
            }
        }
        else {
            // unknown command
            Serial.println("\nUnbekannter Befehl.");
        }

        // flush serial buffer
        Serial.readString();
        Serial << '\n';
    }

    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif

}