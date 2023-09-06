//
// Created by Camleoah on 19.01.2022.
//


#include "Arduino.h"

#include "user_interface.h"
#include "gps_manager.h"
#include "wifi_handler.h"
#include "ram_log.h"
#include "webserial_monitor.h"
#include "version.h"
#include "github_update.h"

void ui_config() {
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
            EEPROM_writeAnything(16 + i, writevalue_ip[i]);
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
        DualSerial.println("\nUngültiger Parameter. Mindestens einer der folgenden Parameter fehlt:");
        DualSerial.println("--ip [ip-adresse]      - ändern der gespeicherten IP-Adresse");
        DualSerial.println("--km [wert]            - ändern des gespeicherten km-Standes'\n");
    }
}

String ui_info() {
    String fw_version = "\nFirmware Version:   " + String(FW_VERSION_MAJOR) + "."+ String(FW_VERSION_MINOR) + "." + String(FW_VERSION_PATCH);
    DualSerial.println(fw_version.c_str());
    DualSerial.print("Wlan Modus:         "); DualSerial.println(wifi_handler_get_mode());
    DualSerial.print("WLan verbunden:     "); DualSerial.println(WiFi.isConnected() ? "ja" : "nein");
    DualSerial.print("IP-Adresse:         "); DualSerial.println(WiFi.localIP().toString());
    DualSerial.print("Aktueller km-Stand: "); DualSerial.println(gpsState.milage_km);
    String str_pos = "Aktuelle Position:  " + String(gpsState.posLat) + "° Nord, " + String(gpsState.posLon) + "° Ost";
    DualSerial.println(str_pos.c_str());
    DualSerial.print("Anzahl Satelliten:  "); DualSerial.println(gpsState.numberSats);
    DualSerial.print("Laufzeit: "); DualSerial.println(ram_log_time_str(millis()));
    ram_log_print_log();
    return fw_version;
}

void ui_debug() {
    char *sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        DualSerial.print("\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n"
               "debug --reboot              - Neustarten des Geräts\n\n");
    }

    else if (!strcmp(sub_key, "--reboot")) {
        DualSerial.println("Starte neu...");
        delay(1000);
        esp_restart();
    }

    else if (!strcmp(sub_key, "--update")) {
        sub_key = strtok(nullptr, " \n");

        if (sub_key == nullptr)
            strcpy(sub_key, "");

        else if (!strcmp(sub_key, "--version")) {

            sub_key = strtok(nullptr, " \n");

            github_update_firmwareUpdate(sub_key);
            Serial.println("Update fehlgeschlagen.");
            return;
        }
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

        else if (!strcmp(rx_command_key, "konfiguriere"))
            ui_config();

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