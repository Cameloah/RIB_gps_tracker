//
// Created by Cameloah on 19.01.2022.
//

#pragma once

/// \brief writes out info such as firmware version
///
/// \return Firmware version string
String ui_info();

/// \brief checks for user input via serial comm and offers commands for accessing internal features
void ui_serial_comm_handler();