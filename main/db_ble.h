/***************************************************************************************************************************
 * @file db_ble.h
 * @brief DroneBridge ESP32 BLE Header file
 *
 * This file is part of DroneBridge and CosmicBridge
 *
 * @author Witty-Wizard <agarwalshashank429@gmail.com>
 * @license Apache License, Version 2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************************************/

#ifndef DB_BLE_H
#define DB_BLE_H

/***************************************************************************************************************************
 * Standard & System Headers
 **************************************************************************************************************************/
#include <stdint.h>

/***************************************************************************************************************************
 * MACROS
 **************************************************************************************************************************/
#define BLE_SVC_SPP_UUID16             0xABF0 ///< 16-bit UUID for the SPP (Serial Port Profile) service.
#define BLE_SVC_SPP_CHR_WRITE_UUID16   0xABF1 ///< 16-bit UUID for the SPP Receive service characteristic.
#define BLE_SVC_SPP_CHR_NOTIFY_UUID16  0xABF2 ///< 16-bit UUID for the SPP Send service characteristic.
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0180 ///< Generic tag appearance value for BLE GAP.
#define BLE_GAP_LE_ROLE_PERIPHERAL     0x00   ///< LE role value indicating a peripheral device.

/***************************************************************************************************************************
 * Public Function Declaration
 **************************************************************************************************************************/

/**
 * @brief Drone Bridge BLE initialisation function
 */
void db_init_ble();

#endif // DB_BLE_H