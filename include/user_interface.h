//
// Created by Cameloah on 19.01.2022.
//

#pragma once

#include "Arduino.h"
#include "gps_manager.h"
#include "ram_log.h"
#include "webserial_monitor.h"
#include "version.h"
#include "github_update.h"
#include "network_manager.h"

/// \brief writes out info such as firmware version
///
/// \return Firmware version string
String ui_info();

/// \brief checks for user input via serial comm and offers commands for accessing internal features
void ui_serial_comm_handler();