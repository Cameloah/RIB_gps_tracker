#include <Arduino.h>
#include <cstdio>
#include "ESPAsyncWebServer.h"

#include "wifi_handler.h"
#include "gps_manager.h"
#include "version.h"
#include "github_update.h"
#include "tools/loop_timer.h"
#include "user_interface.h"
#include "ram_log.h"

/* Changelog:
 * - 1.1.2 added ram log and improved faulty measurement rejection
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


void setup() {
    delay(1000);
    // Setup DualSerial communication
    DualSerial.begin(115200);

    // init eeprom flash
    DualSerial.println("Initialisiere Speichermodul...");
    while (!EEPROM.begin(EEPROM_SIZE)) {}
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
    retval = wifi_handler_init("Kanustation", "kanustation", ip_default.c_str(), "192.168.2.1", "255.255.255.0",
                                URL_FW_VERSION, URL_FW_BIN);
    if(retval == WIFI_HANDLER_ERROR_NO_ERROR) {
        DualSerial.println("Suche nach Updates...");
        retval = github_update_fwVersionCheck(FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
        if (retval == GITHUB_UPDATE_ERROR_NO_ERROR)
            github_update_firmwareUpdate();
        else if (retval == GITHUB_UPDATE_ERROR_NO_UPDATE)
            DualSerial.println("FW ist aktuell!");
        else {
            DualSerial.println("Fehler.");
            ram_log_notify(RAM_LOG_ERROR_GITHUB_UPDATE, retval);
        }

        // since we have wifi, lets start the server
        ram_log_notify(RAM_LOG_INFO, "Starte Server", true);
        server.on("/", HTTP_GET, handleRoot);
        server.begin();
    }
    else if (retval == WIFI_HANDLER_ERROR_CONNECT) {
        DualSerial.println("WLAN nicht gefunden.");
        ram_log_notify(RAM_LOG_ERROR_WIFI_HANDLER, retval);
    }
    else {
        ram_log_notify(RAM_LOG_ERROR_WIFI_HANDLER, retval);
        DualSerial.println("Fehler."); }

    // gps setup
    gps_manager_init();

    DualSerial.println("Einsatzbereit!");
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
            ram_log_notify(RAM_LOG_INFO, "WiFi wieder verbunden. Starte Server.", true);
            server.on("/", handleRoot);
            server.begin();
        }
        else if (retval == WIFI_HANDLER_ERROR_CONNECT) {
            DualSerial.println("WLAN nicht gefunden.");
            delay(10000);

        }
        else {
            DualSerial.println("Fehler bei WLAN-Suche.");
            ram_log_notify(RAM_LOG_ERROR_WIFI_HANDLER, retval);
        }
    }

    // always request gps and count milage
    gps_manager_update();

    // listen for user input
    ui_serial_comm_handler();

    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif

}