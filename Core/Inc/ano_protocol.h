/**
 ******************************************************************************
 * @file           : ano_protocol.h
 * @brief          : ANO Flight Controller Protocol Encoder
 ******************************************************************************
 * @attention
 *
 * ANO Protocol Frame Format:
 * [0xAA][0xFF][Frame ID][Length][Data...][SUM_CHECK][ADD_CHECK]
 *
 * Supported Frames:
 *   - Frame 0x51: Optical Flow (Mode 1, 7 bytes data)
 *   - Frame 0x34: Distance Sensor (7 bytes data)
 *
 ******************************************************************************
 */

#ifndef __ANO_PROTOCOL_H
#define __ANO_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ANO Protocol Constants */
#define ANO_FRAME_HEADER1 0xAA // Frame header byte 1
#define ANO_FRAME_HEADER2 0xFF // Frame header byte 2

/* ANO Frame IDs */
#define ANO_FRAME_ID_DISTANCE 0x34     // Distance frame (7 bytes data)
#define ANO_FRAME_ID_OPTICAL_FLOW 0x51 // Optical flow frame (variable length)

/* ANO Frame Lengths */
#define ANO_HEADER_LEN 4          // AA + FF + ID + LEN
#define ANO_CHECKSUM_LEN 2        // SUM_CHECK + ADD_CHECK
#define ANO_DISTANCE_DATA_LEN 7   // Distance frame data length
#define ANO_FLOW_MODE1_DATA_LEN 7 // Optical flow Mode 1 data length
#define ANO_MAX_FRAME_LEN 32      // Maximum frame length

/* Optical Flow Modes */
#define ANO_FLOW_MODE_0 0 // Raw flow data (DX_0, DY_0 as int8)
#define ANO_FLOW_MODE_1 1 // Fused flow data (DX_1, DY_1 as int16)
#define ANO_FLOW_MODE_2 2 // IMU-fused flow (with integrated position)

/**
 * @brief ANO Distance Frame Data (Frame ID: 0x34, 7 bytes payload)
 * 参考: 发送示例.txt - Send_OF_DIST 函数
 * 格式: [Direction(1)][Angle(2)][Distance(4)]
 */
typedef struct __attribute__((packed)) {
  uint8_t direction;    // Byte 0: 安装方向 (0:向下)
  uint16_t angle;       // Byte 1-2: 角度信息 (通常为0)
  uint32_t distance_cm; // Byte 3-6: 距离信息 (单位: CM, 4字节)
} ano_distance_data_t;

/**
 * @brief ANO Optical Flow Frame Data (Frame ID: 0x51, Mode 1, 7 bytes)
 * Manual V3.2 Page 11
 * Format: [Mode][State][DX_L][DX_H][DY_L][DY_H][Quality]
 */
typedef struct __attribute__((packed)) {
  uint8_t mode;    // Byte 0: Flow mode (Fixed to 1)
  uint8_t state;   // Byte 1: Status (0:Invalid, 1:Valid) <-- 新增此成员
  int16_t dx;      // Byte 2-3: Flow X velocity (cm/s)
  int16_t dy;      // Byte 4-5: Flow Y velocity (cm/s)
  uint8_t quality; // Byte 6: Flow quality (0-255)      <-- 移动到最后
} ano_optical_flow_mode1_t;

/**
 * @brief ANO Frame Structure
 */
typedef struct {
  uint8_t header1;                 // 0xAA
  uint8_t header2;                 // 0xFF
  uint8_t frame_id;                // Frame ID
  uint8_t length;                  // Data length
  uint8_t data[ANO_MAX_FRAME_LEN]; // Data payload
  uint8_t sum_check;               // SUM checksum
  uint8_t add_check;               // ADD checksum
} ano_frame_t;

/**
 * @brief Calculate ANO checksum
 * @param data: Pointer to data buffer (starting from Frame ID)
 * @param length: Length of data (Frame ID + Length + Data bytes)
 * @param sum_check: Pointer to store SUM checksum
 * @param add_check: Pointer to store ADD checksum
 */
void ano_calculate_checksum(const uint8_t *data, uint8_t length,
                            uint8_t *sum_check, uint8_t *add_check);

/**
 * @brief Send ANO distance frame
 * @param huart: UART handle for transmission
 * @param distance_data: Pointer to distance data structure
 * @retval HAL status
 */
HAL_StatusTypeDef ano_send_distance(UART_HandleTypeDef *huart,
                                    const ano_distance_data_t *distance_data);

/**
 * @brief Send ANO optical flow frame (Mode 1)
 * @param huart: UART handle for transmission
 * @param flow_data: Pointer to optical flow data structure
 * @retval HAL status
 */
HAL_StatusTypeDef
ano_send_optical_flow_mode1(UART_HandleTypeDef *huart,
                            const ano_optical_flow_mode1_t *flow_data);

/**
 * @brief Send ANO distance frame via DMA
 * @param huart: UART handle for transmission
 * @param distance_data: Pointer to distance data structure
 * @retval HAL status
 */
HAL_StatusTypeDef
ano_send_distance_dma(UART_HandleTypeDef *huart,
                      const ano_distance_data_t *distance_data);

/**
 * @brief Send ANO optical flow frame (Mode 1) via DMA
 * @param huart: UART handle for transmission
 * @param flow_data: Pointer to optical flow data structure
 * @retval HAL status
 */
HAL_StatusTypeDef
ano_send_optical_flow_mode1_dma(UART_HandleTypeDef *huart,
                                const ano_optical_flow_mode1_t *flow_data);

#ifdef __cplusplus
}
#endif

#endif /* __ANO_PROTOCOL_H */
