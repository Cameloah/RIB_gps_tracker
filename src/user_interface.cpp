//
// Created by koorj on 19.01.2022.
//


#include "Arduino.h"

#include "user_interface.h"
#include "gps_manager.h"
#include "wifi_handler.h"
#include "ram_log.h"
#include "webserial_monitor.h"
#include "version.h"

String ui_info() {
    String fw_version = "\nFirmware Version:   " + String(FW_VERSION_MAJOR) + "."+ String(FW_VERSION_MINOR) + "." + String(FW_VERSION_PATCH);
    DualSerial.println(fw_version.c_str());
    DualSerial.print("WLan verbunden:     "); DualSerial.println(WiFi.isConnected() ? "ja" : "nein");
    DualSerial.print("IP-Adresse:         "); DualSerial.println(WiFi.localIP().toString());
    ram_log_print_log();
    return fw_version;
}

void ui_debug() {
    char *sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        DualSerial.print("\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n"
               "debug --reboot              - Neustarten des Geräts\n\n");
    } else if (!strcmp(sub_key, "--reboot")) {
        DualSerial.println("Starte neu...");
        delay(1000);
        esp_restart();
    }

    else {
        DualSerial.print("\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n"
               "debug --reboot              - Neustarten des Geräts\n\n");
    }
}

void ui_serial_comm_handler() {
    // listen for user input
    if (DualSerial.available())
        delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = DualSerial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        DualSerial.readBytes(rx_user_input, rx_available_bytes);

        // extract first word as command key
        char *rx_command_key = strtok(rx_user_input, " \n");

        // catch exception where no token was sent
        if (rx_command_key == nullptr)
            return;

        else if (!strcmp(rx_command_key, "info"))
            ui_info();

        else if (!strcmp(rx_command_key, "debug"))
            ui_debug();

        else {
            // unknown command
            DualSerial.println("\nUnbekannter Befehl.");
        }

        // flush serial buffer
        DualSerial.readString();

        DualSerial << '\n';
    }
}