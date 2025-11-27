/**
 ******************************************************************************
 * @file           : protocol_bridge.h
 * @brief          : MAVLINK to ANO Protocol Data Conversion Bridge
 ******************************************************************************
 * @attention
 *
 * Data conversion between:
 *   - MAVLINK PX4 Optical Flow (radians) -> ANO Optical Flow Mode 1 (scaled int16)
 *   - MAVLINK PX4 Distance Sensor (cm) -> ANO Distance (mm)
 *
 ******************************************************************************
 */

#ifndef __PROTOCOL_BRIDGE_H
#define __PROTOCOL_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mavlink_parser.h"
#include "ano_protocol.h"

/* Conversion scaling factors */
#define FLOW_RAD_TO_ANO_SCALE   1000.0f     // Scale radians to int16 (adjust as needed)
#define DISTANCE_CM_TO_MM       10          // Centimeters to millimeters

/**
 * @brief Convert MAVLINK optical flow to ANO optical flow Mode 1
 * @param mavlink_flow: Pointer to MAVLINK optical flow data
 * @param ano_flow: Pointer to ANO optical flow structure to fill
 * @retval true on successful conversion
 *
 * Conversion:
 * - integrated_x (rad) -> dx (int16, scaled)
 * - integrated_y (rad) -> dy (int16, scaled)
 * - quality (0-255) -> quality (0-255)
 * - mode set to 1 (Mode 1: fused flow data)
 */
bool bridge_convert_optical_flow(const mavlink_optical_flow_rad_t *mavlink_flow,
                                  ano_optical_flow_mode1_t *ano_flow);

/**
 * @brief Convert MAVLINK distance sensor to ANO distance
 * @param mavlink_distance: Pointer to MAVLINK distance sensor data
 * @param ano_distance: Pointer to ANO distance structure to fill
 * @retval true on successful conversion
 *
 * Conversion:
 * - current_distance (cm) -> distance_mm (mm)
 * - orientation -> direction
 * - Status and angle set based on sensor data
 */
bool bridge_convert_distance(const mavlink_distance_sensor_t *mavlink_distance,
                              ano_distance_data_t *ano_distance);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_BRIDGE_H */
