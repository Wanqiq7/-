/**
 ******************************************************************************
 * @file           : mavlink_parser.h
 * @brief          : MAVLINK PX4 Protocol Parser
 ******************************************************************************
 * @attention
 *
 * MAVLINK v1 Parser for PX4 Optical Flow Sensor
 * Supports:
 *   - Message ID 0x6A (106): OPTICAL_FLOW_RAD (44 bytes payload)
 *   - Message ID 0x84 (132): DISTANCE_SENSOR (14 bytes payload)
 *
 ******************************************************************************
 */

#ifndef __MAVLINK_PARSER_H
#define __MAVLINK_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* MAVLINK Constants */
#define MAVLINK_STX                 0xFE    // MAVLINK v1 start byte
#define MAVLINK_SYSID               0x00    // System ID
#define MAVLINK_COMPID              0x9E    // Component ID

/* MAVLINK Message IDs */
#define MAVLINK_MSG_ID_OPTICAL_FLOW_RAD     106  // 0x6A
#define MAVLINK_MSG_ID_DISTANCE_SENSOR      132  // 0x84

/* MAVLINK Payload Lengths */
#define MAVLINK_OPTICAL_FLOW_RAD_LEN    44
#define MAVLINK_DISTANCE_SENSOR_LEN     14

/* MAVLINK Frame Length */
#define MAVLINK_HEADER_LEN              6   // STX + LEN + SEQ + SYSID + COMPID + MSGID
#define MAVLINK_CHECKSUM_LEN            2   // CRC_LOW + CRC_HIGH
#define MAVLINK_MAX_PAYLOAD_LEN         255

/* Parser Buffer Size */
#define MAVLINK_MAX_FRAME_LEN   (MAVLINK_HEADER_LEN + MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_CHECKSUM_LEN)

/**
 * @brief MAVLINK Parser State Machine
 */
typedef enum {
    MAVLINK_PARSE_STATE_IDLE = 0,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1
} mavlink_parse_state_t;

/**
 * @brief MAVLINK PX4 Optical Flow Message (0x6A, 44 bytes)
 *
 * Integration time: Microseconds
 * Integrated flow: Radians
 * Distance: Meters
 * Temperature: Centi-degrees Celsius
 */
typedef struct __attribute__((packed)) {
    uint64_t    time_usec;              // Timestamp (microseconds)
    uint32_t    integration_time_us;    // Integration time (microseconds)
    float       integrated_x;           // Integrated flow X (radians)
    float       integrated_y;           // Integrated flow Y (radians)
    float       integrated_xgyro;       // Integrated gyro X (radians)
    float       integrated_ygyro;       // Integrated gyro Y (radians)
    float       integrated_zgyro;       // Integrated gyro Z (radians)
    uint32_t    time_delta_distance_us; // Time since distance measurement (us)
    float       distance;               // Ground distance (meters)
    uint16_t    temperature;            // Temperature (centi-degrees C)
    uint8_t     sensor_id;              // Sensor ID
    uint8_t     quality;                // Optical flow quality (0-255)
} mavlink_optical_flow_rad_t;

/**
 * @brief MAVLINK PX4 Distance Sensor Message (0x84, 14 bytes)
 *
 * All distances in centimeters
 */
typedef struct __attribute__((packed)) {
    uint32_t    time_boot_ms;       // Timestamp (milliseconds)
    uint16_t    min_distance;       // Minimum distance (cm)
    uint16_t    max_distance;       // Maximum distance (cm)
    uint16_t    current_distance;   // Current distance (cm)
    uint8_t     type;               // Distance sensor type
    uint8_t     id;                 // Sensor ID
    uint8_t     orientation;        // Sensor orientation
    uint8_t     covariance;         // Measurement covariance
} mavlink_distance_sensor_t;

/**
 * @brief MAVLINK Parser Context
 */
typedef struct {
    mavlink_parse_state_t   state;              // Parser state
    uint8_t                 payload_len;        // Expected payload length
    uint8_t                 seq;                // Sequence number
    uint8_t                 sysid;              // System ID
    uint8_t                 compid;             // Component ID
    uint8_t                 msgid;              // Message ID
    uint8_t                 payload[MAVLINK_MAX_PAYLOAD_LEN];  // Payload buffer
    uint16_t                payload_idx;        // Current payload index
    uint16_t                checksum;           // Calculated checksum
    uint8_t                 crc_low;            // Received CRC low byte
    uint8_t                 crc_high;           // Received CRC high byte

    // Parsed message flags
    bool                    optical_flow_ready; // Optical flow message ready
    bool                    distance_ready;     // Distance message ready

    // Parsed messages
    mavlink_optical_flow_rad_t  optical_flow;   // Optical flow data
    mavlink_distance_sensor_t   distance_sensor; // Distance sensor data
} mavlink_parser_t;

/**
 * @brief Initialize MAVLINK parser
 * @param parser: Pointer to parser context
 */
void mavlink_parser_init(mavlink_parser_t *parser);

/**
 * @brief Parse a single byte from MAVLINK stream
 * @param parser: Pointer to parser context
 * @param byte: Byte to parse
 * @retval true if a complete message was received
 */
bool mavlink_parse_char(mavlink_parser_t *parser, uint8_t byte);

/**
 * @brief Get optical flow data if available
 * @param parser: Pointer to parser context
 * @param flow: Pointer to optical flow structure to fill
 * @retval true if new data is available
 */
bool mavlink_get_optical_flow(mavlink_parser_t *parser, mavlink_optical_flow_rad_t *flow);

/**
 * @brief Get distance sensor data if available
 * @param parser: Pointer to parser context
 * @param distance: Pointer to distance sensor structure to fill
 * @retval true if new data is available
 */
bool mavlink_get_distance(mavlink_parser_t *parser, mavlink_distance_sensor_t *distance);

/**
 * @brief Compute CRC-16-MCRF4XX checksum for MAVLINK
 * @param data: Pointer to data buffer
 * @param length: Length of data
 * @param crc_extra: CRC extra byte for message ID
 * @retval Computed CRC-16 checksum
 */
uint16_t mavlink_crc_compute(const uint8_t *data, uint16_t length, uint8_t crc_extra);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_PARSER_H */
