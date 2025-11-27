/**
 ******************************************************************************
 * @file           : protocol_bridge.c
 * @brief          : MAVLINK to ANO Protocol Data Conversion Implementation
 ******************************************************************************
 * @attention
 *
 * Data conversion implementation between MAVLINK PX4 and ANO protocols
 *
 ******************************************************************************
 */

#include "protocol_bridge.h"
#include <math.h>
#include <string.h>

/**
 * @brief Convert MAVLINK optical flow to ANO optical flow Mode 1
 * @param mavlink_flow: Pointer to MAVLINK optical flow data
 * @param ano_flow: Pointer to ANO optical flow structure to fill
 * @retval true on successful conversion
 *
 * Conversion Details:
 * - MAVLINK integrated_x/y are in radians (accumulated flow over integration time)
 * - ANO expects flow velocity as int16 values
 * - We scale radians by FLOW_RAD_TO_ANO_SCALE (1000.0) to get reasonable int16 values
 * - Quality maps directly (0-255)
 */
bool bridge_convert_optical_flow(const mavlink_optical_flow_rad_t *mavlink_flow,
                                  ano_optical_flow_mode1_t *ano_flow)
{
    if (mavlink_flow == NULL || ano_flow == NULL) {
        return false;
    }

    // Clear output structure
    memset(ano_flow, 0, sizeof(ano_optical_flow_mode1_t));

    // Set mode to 1 (fused flow data)
    ano_flow->mode = ANO_FLOW_MODE_1;

    // Convert quality (direct mapping)
    ano_flow->quality = mavlink_flow->quality;

    // Convert integrated flow (radians) to scaled int16
    // MAVLINK: integrated_x/y in radians (accumulated over integration_time_us)
    // ANO: dx/dy as int16 scaled values
    //
    // Scaling: Multiply by 1000 to convert rad to milli-radians for better resolution
    float dx_scaled = mavlink_flow->integrated_x * FLOW_RAD_TO_ANO_SCALE;
    float dy_scaled = mavlink_flow->integrated_y * FLOW_RAD_TO_ANO_SCALE;

    // Clamp to int16 range [-32768, 32767]
    if (dx_scaled > 32767.0f) dx_scaled = 32767.0f;
    if (dx_scaled < -32768.0f) dx_scaled = -32768.0f;
    if (dy_scaled > 32767.0f) dy_scaled = 32767.0f;
    if (dy_scaled < -32768.0f) dy_scaled = -32768.0f;

    ano_flow->dx = (int16_t)dx_scaled;
    ano_flow->dy = (int16_t)dy_scaled;

    // Reserved field
    ano_flow->reserved = 0;

    return true;
}

/**
 * @brief Convert MAVLINK distance sensor to ANO distance
 * @param mavlink_distance: Pointer to MAVLINK distance sensor data
 * @param ano_distance: Pointer to ANO distance structure to fill
 * @retval true on successful conversion
 *
 * Conversion Details:
 * - MAVLINK current_distance in centimeters
 * - ANO distance_mm in millimeters (multiply by 10)
 * - Direction mapping based on orientation field
 * - Angle set to 0 (downward facing sensor assumed)
 */
bool bridge_convert_distance(const mavlink_distance_sensor_t *mavlink_distance,
                              ano_distance_data_t *ano_distance)
{
    if (mavlink_distance == NULL || ano_distance == NULL) {
        return false;
    }

    // Clear output structure
    memset(ano_distance, 0, sizeof(ano_distance_data_t));

    // Convert distance from cm to mm
    uint32_t distance_mm = (uint32_t)mavlink_distance->current_distance * DISTANCE_CM_TO_MM;

    // Clamp to uint16 range [0, 65535] mm
    if (distance_mm > 65535) {
        distance_mm = 65535;
    }

    ano_distance->distance_mm = (uint16_t)distance_mm;

    // Map MAVLINK orientation to ANO direction
    // MAVLINK orientation: 0=forward, 1=right, 2=back, 3=left, 4=up, 5=down
    // ANO direction: typically 0=down, 1=forward
    switch (mavlink_distance->orientation) {
        case 5:  // MAVLINK: down
            ano_distance->direction = 0;  // ANO: down
            ano_distance->angle = 0;      // 0 degrees (downward)
            break;
        case 0:  // MAVLINK: forward
            ano_distance->direction = 1;  // ANO: forward
            ano_distance->angle = 90;     // 90 degrees (forward)
            break;
        default:
            ano_distance->direction = mavlink_distance->orientation;
            ano_distance->angle = 0;
            break;
    }

    // Set status based on covariance (0 = good, higher = worse)
    // ANO status: 0 = valid, 1 = invalid (simple mapping)
    if (mavlink_distance->covariance < 100) {
        ano_distance->status = 0;  // Valid measurement
    } else {
        ano_distance->status = 1;  // Invalid measurement
    }

    // Reserved fields
    ano_distance->reserved1 = 0;
    ano_distance->reserved2 = 0;

    return true;
}
