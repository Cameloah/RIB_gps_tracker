//
// Created by Cameloah on 19.01.2022.
//

#include "user_interface.h"


void ui_error() {
    DualSerial.print("\nUngültiger Befehl. Mindestens einer der folgenden Parameter fehlt:\n");
}

void ui_config_help() {
    DualSerial.print("konfiguriere --ip [ip-adresse]      - ändern der gespeicherten IP-Adresse\n"
                     "konfiguriere --km [wert]            - ändern des gespeicherten km-Standes\n"
                     "konfiguriere --alarm [0/1]          - aktivieren/deaktivieren des Geschwindigkeitsalarms\n\n");
}

void ui_config() {
    char *sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        ui_error();
        ui_config_help();
        return;
    }

    if (!strcmp(sub_key, "--ip")) {

        sub_key = strtok(nullptr, "\n");
        if (sub_key == nullptr) {
            DualSerial.println("Fehler.");
            return;
        }
        
        (wifi_config.set("localIP", String(sub_key), true));
        
        DualSerial.println("Speichere IP " + String(sub_key) + "...");
        DualSerial.println("Starte neu...");
        delay(1000);
        esp_restart();
    } 
    
    else if (!strcmp(sub_key, "--km")) {
        sub_key = strtok(nullptr, " \n");
        if (sub_key == nullptr) {
            DualSerial.println("Fehler.");
            return;
        }

        gpsState.mileage_km = atof(sub_key);
        DualSerial.println(atof(sub_key));
        gps_parameters.set("mileage_km", (double) atof(sub_key), true);

        DualSerial.println("Speichere neuen Kilometerstand...");
        return;
    } 

    else if (!strcmp(sub_key, "--alarm")) {
        sub_key = strtok(nullptr, " \n");
        if (sub_key == nullptr) {
            DualSerial.println("Fehler.");
            return;
        }

        gps_parameters.set("speedlimit_on", (bool) atoi(sub_key), true);

        DualSerial.println("Geschwindigkeitsalarm: " + String((bool) atoi(sub_key)) + "...");
        return;
    } 
    
    else ui_error();

    ui_config_help();
}

void ui_info_help() {
    DualSerial.println("info                                - Laufzeitinformationen über das Gerät");
}

String ui_info() {
    String fw_version;
    String info;

    // Construct the firmware version string
    fw_version += "\nFirmware-Version:      v"
    + String(FW_VERSION_MAJOR) + "." + String(FW_VERSION_MINOR) + "." + String(FW_VERSION_PATCH) + "\n";

    // get reset reason
    String boot_msg = String(esp_reset_reason(), HEX);
    if (boot_msg.length() == 1)
        boot_msg = "0" + boot_msg;
    boot_msg = "0x" + boot_msg;

    // Append additional system information to the info string.
    info = "Wlan Modus:            " + network_manager_get_mode() + "\n"
    + "WLan verbunden:        " + (WiFi.isConnected() ? WiFi.SSID() : "not connected") + "\n"
    + "IP-Adresse:            " + (WiFi.isConnected() ? WiFi.localIP().toString() : "") + "\n"
    + "Aktueller km-Stand:    " + String(gpsState.mileage_km) + "\n"
    + "Aktuelle Position:     " + String(gpsState.posLat) + "° Nord, " + String(gpsState.posLon) + "° Ost" + "\n"
    + "Anzahl Satelliten:     " + String(gpsState.numberSats) + "\n"
    + "Laufzeit:              " + ram_log_time_str(esp_timer_get_time()) + "\n"
    + "Boot Nachicht:         " + boot_msg + "\n\n";

    DualSerial.print(fw_version + info);

    ram_log_print_log();

    return fw_version;
}

void ui_debug_help() {
    DualSerial.print("debug --reboot                      - Neustarten des Geräts\n"
                     "      --update --version [vX.X.X]   - Update/Downgrade auf gewünschte Version\n\n");
}

void ui_debug() {
    char *sub_key = strtok(nullptr, " \n");

    if (sub_key == nullptr) {
        ui_error();
        ui_debug_help();
        return;
    } else if (!strcmp(sub_key, "--reboot")) {
        DualSerial.println("Starte neu...");
        delay(1000);
        esp_restart();
    } else if (!strcmp(sub_key, "--update")) {
        sub_key = strtok(nullptr, " \n");

        if (sub_key == nullptr)
            strcpy(sub_key, "");

        else if (!strcmp(sub_key, "--version")) {

            sub_key = strtok(nullptr, " \n");

            github_update_firmwareUpdate(sub_key);
            Serial.println("Update fehlgeschlagen.");
            return;
        }
    } else
        ui_error();
    ui_debug_help();
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

        else if (!strcmp(rx_command_key, "hilfe")) {
            DualSerial.println("\nVerfügbare Befehle:");
            ui_info_help();
            ui_config_help();
            ui_debug_help();
        } else {
            // unknown command
            DualSerial.println("\nUnbekannter Befehl. Benutzen Sie 'hilfe' um verfügbare Befehle anzuzeigen.");
        }

        // flush serial buffer
        DualSerial.readString();

        DualSerial << '\n';
    }
}