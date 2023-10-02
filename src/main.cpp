#include <Arduino.h>
#include "ESPAsyncWebServer.h"

#include "wifi_handler.h"
#include "gps_manager.h"
#include "github_update.h"
#include "tools/loop_timer.h"
#include "user_interface.h"
#include "ram_log.h"


// debug and system control options
#define SYSCTRL_LOOPTIMER               // enable loop frequency control, remember to also set the loop freq in the loop_timer.h
#define INTERVAL_WIFI_CHECK_MS          60000  // how often we search for Wi-Fi

// display the current mileage on root
void handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", String(gpsState.milage_km));
}


void setup() {
    delay(1000);
    // Setup USB comm + Web-serial
    DualSerial.begin(115200);

    // init eeprom flash
    DualSerial.println("Initialisiere Speichermodul...");
    while (!EEPROM.begin(EEPROM_SIZE)) {}
    DualSerial.println("Erfolgreich.");

    // Wi-Fi setup
    DualSerial.println("Starte Wifi...");
    wifi_info_buffer.ap_name = "Neuer GPS-Tracker";             // name of the AP when not configured
    wifi_info_buffer.device_name = "GPS Boot ";                 // name of the AP when configured
    uint8_t retval = wifi_handler_init();

    if (retval == WIFI_HANDLER_ERROR_NO_ERROR) {
        DualSerial.println("Suche nach Updates...");
        retval = github_update_checkforlatest();
        if (retval == GITHUB_UPDATE_ERROR_NO_ERROR)
            github_update_firmwareUpdate();                     // we found an update, lets update!
        else if (retval == GITHUB_UPDATE_ERROR_NO_UPDATE)
            DualSerial.println("FW ist aktuell!");
        else {
            DualSerial.println("Fehler.");
            ram_log_notify(RAM_LOG_ERROR_GITHUB_UPDATE, retval);
        }

        // since we have Wi-Fi, lets start the server
        ram_log_notify(RAM_LOG_INFO, "Starte Server", true);
        server.on("/", HTTP_GET, handleRoot);
        server.begin();
    } else if (retval == WIFI_HANDLER_ERROR_CONNECT) {
        DualSerial.println("WLAN nicht gefunden.");
        ram_log_notify(RAM_LOG_ERROR_WIFI_HANDLER, retval);
    } else {
        ram_log_notify(RAM_LOG_ERROR_WIFI_HANDLER, retval);
        DualSerial.println("Fehler.");
    }

    // gps module setup
    gps_manager_init();

    DualSerial.println("Einsatzbereit!");
}

// counting loop iterations to search for Wi-Fi
double counter_wifi = 0;

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    // we keep track of the loop iterations and only check in a specified time interval
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
        } else if (retval == WIFI_HANDLER_ERROR_CONNECT) {
            DualSerial.println("WLAN nicht gefunden.");

        } else {
            DualSerial.println("Fehler bei WLAN-Suche.");
            ram_log_notify(RAM_LOG_ERROR_WIFI_HANDLER, retval);
        }
    }

    // always request gps and count mileage
    gps_manager_update();

    // run Wi-Fi server routine
    wifi_handler_update();

    // listen for user input
    ui_serial_comm_handler();

    // iterate loop timer to track loop frequency
    loop_timer++;

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif

}