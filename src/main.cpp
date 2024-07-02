#include "main.h"

// debug and system control options


// display the current mileage on root
void handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", String(gpsState.mileage_km));
}


void setup() {
    delay(1000);
    // Setup USB comm + Web-serial
    DualSerial.begin(115200);

    // Wi-Fi setup
    DualSerial.println("Starte Wifi...");

    server.on("/", HTTP_GET, handleRoot);
    project_utils_init("Neuer GPS-Tracker"); 

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
    if (!network_manager_is_connected() && (counter_wifi * 1000 / FREQ_LOOP_CYCLE_HZ > INTERVAL_WIFI_CHECK_MS)) {
        counter_wifi = 0;
        network_manager_wifi_connect();
    }

    // always request gps and count mileage
    gps_manager_update();

    // run Wi-Fi server routine
    project_utils_update();

    // listen for user input
    ui_serial_comm_handler();

    // iterate loop timer to track loop frequency
    loop_timer++;

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif

}