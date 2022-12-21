#include <Arduino.h>
#include <cstdio>
#include "ESPAsyncWebServer.h"

#include "wifi_handler.h"
#include "gps_manager.h"
#include "version.h"
#include "github_update.h"
#include "tools/loop_timer.h"
#include "ram_log.h"

/* Changelog:
 * - 1.1.0 added webserial support using an asynchronous web server
 *
 * - 1.0.0 basic readout adapted from adafruit mpu6050 example
 *      display data and interface via DualSerial comm
 *      calculates rot matrices from sensor to device housing and from device to ship during calibration
*/

// debug and system control options
#define SYSCTRL_LOOPTIMER               // enable loop frequency control, remember to also set the loop freq in the loop_timer.h
#define INTERVAL_WIFI_CHECK_MS          60000

String ip_default = "192.168.2.73";

void handleRoot(AsyncWebServerRequest *request)
{request->send(200, "text/html",String(gpsState.milage_km));
}


String ui_info() {
    String fw_version = "\nRIB-GPS-Tracker Version: ";
    fw_version.concat(FW_VERSION_MAJOR);
    fw_version.concat(".");
    fw_version.concat(FW_VERSION_MINOR);
    fw_version.concat(".");
    fw_version.concat(FW_VERSION_PATCH);
    fw_version.concat("\n");
    DualSerial.print(fw_version.c_str());
    DualSerial.print("WLan verbunden: "); DualSerial.println(WiFi.isConnected());
    ram_log_print_log();
    return fw_version;
}

void setup() {
    delay(1000);
    // Setup DualSerial communication
    DualSerial.begin(115200);
    ram_log_notify(RAM_LOG_INFO, "Start up.");

    // init eeprom flash
    DualSerial.println("Initialisiere Speichermodul...");
    while (!EEPROM.begin(EEPROM_SIZE)) {
        continue;
    }
    DualSerial.println("Erfolgreich.");

    // wifi setup
    uint8_t retval = WIFI_HANDLER_ERROR_UNKNOWN;
    DualSerial.println("Starte Wifi.");

    // read ip from flash
    uint8_t readValue_ip[4];
    for (int i = 0; i < 4; ++i) {
        EEPROM_readAnything(16+i, readValue_ip[i]);
    }

    if (readValue_ip[0] == 192) {
        char ip_loaded[15];
        sprintf( ip_loaded, "%d.%d.%d.%d", readValue_ip[0], readValue_ip[1], readValue_ip[2], readValue_ip[3]);
        strcpy((char*)ip_default.c_str(), ip_loaded);
        DualSerial.print("IP-Adresse aus Speicher geladen: ");
    }
    else DualSerial.print("Verwende Standard-IP: ");

    DualSerial.println(ip_default);

    // start wifi
    retval = wifi_handler_init("WLAN", "kanustation", ip_default.c_str(), "192.168.2.1", "255.255.255.0",
                                URL_FW_VERSION, URL_FW_BIN);
    if(retval == WIFI_HANDLER_ERROR_NO_ERROR) {
        DualSerial.println("Suche nach Updates...");
        retval = github_update_fwVersionCheck(FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
        if (retval == WIFI_HANDLER_ERROR_NO_ERROR)
            github_update_firmwareUpdate();
        else if (retval == WIFI_HANDLER_ERROR_NO_UPDATE)
            DualSerial.println("FW ist aktuell!");
        else DualSerial.println("Fehler.");

        // since we have wifi, lets start the server
        DualSerial.println("Starte Server");
        server.on("/", HTTP_GET, handleRoot);
        server.begin();
    }
    else if (retval == WIFI_HANDLER_ERROR_WIFI)
        DualSerial.println("WLAN nicht gefunden.");
    else DualSerial.println("Fehler.");

    // gps setup
    gps_manager_init();
}


double counter_wifi = 0;

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    counter_wifi++;
    if (!wifi_handler_is_connected() && (counter_wifi * 1000 / FREQ_LOOP_CYCLE_HZ > INTERVAL_WIFI_CHECK_MS)) {
        counter_wifi = 0;

        // try to reconnect
        uint8_t retval = wifi_handler_connect();
        if (retval == WIFI_HANDLER_ERROR_NO_ERROR) {
            // setup root callback to send data
            DualSerial.println("WiFi wieder verbunden. Starte Server.");
            server.on("/", handleRoot);
            server.begin();
        }
        if (retval == WIFI_HANDLER_ERROR_WIFI) {
            DualSerial.println("WLAN nicht gefunden.");
            delay(10000);

        }
        else DualSerial.println("Fehler.");
    }

    // always request gps and count milage
    gps_manager_update();

    // handle DualSerial commands
    // listen for user input
    if (DualSerial.available())
        delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = DualSerial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        DualSerial.readBytes(rx_user_input, rx_available_bytes);

        // extract first word as command key
        char* rx_command_key = strtok(rx_user_input, " \n");

        // catch exception where no token was sent
        if (rx_command_key == nullptr)
            return;

        else if (!strcmp(rx_command_key, "info"))
            ui_info();

        else if (!strcmp(rx_command_key, "konfiguriere")) {
            char *sub_key = strtok(nullptr, " \n");

            if (sub_key == nullptr) {
                DualSerial.println("\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:");
                DualSerial.println("--ip [ip-adresse]      - ändern der gespeicherten IP-Adresse");
                DualSerial.println("--km [wert]            - ändern des gespeicherten km-Standes'\n");
                return;
            }

            if (!strcmp(sub_key, "--ip")) {

                uint8_t writevalue_ip[4];

                for (int i = 0; i < 4; ++i) {
                    char *sub_key = strtok(nullptr, ". \n");
                    if (sub_key == nullptr) {
                        DualSerial.println("Fehler.");
                        return;
                    }
                    DualSerial.print(writevalue_ip[i] = atoi(sub_key));
                    DualSerial.println(" ");
                }

                for (int i = 0; i < 4; ++i) {
                    EEPROM_writeAnything(16+i, writevalue_ip[i]);
                }
                EEPROM.commit();
                DualSerial.println("Speichere IP...");
                DualSerial.println("Starte neu...");
                delay(1000);
                esp_restart();
            }

            else if (!strcmp(sub_key, "--km")) {
                char *sub_key = strtok(nullptr, " \n");
                gpsState.milage_km = atof(sub_key);
                long writeValue = gpsState.milage_km * 1000000;
                EEPROM_writeAnything(12, writeValue);
                EEPROM.commit(); // commit data to flash

                DualSerial.println("Speichere neuen Kilometerstand...");
            }

            else {
                // unknown command
                DualSerial.println("\nUnbekannter Befehl.");
            }
        }
        else {
            // unknown command
            DualSerial.println("\nUnbekannter Befehl.");
        }

        // flush DualSerial buffer
        DualSerial.readString();
        DualSerial << '\n';
    }

    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif

}