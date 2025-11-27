/**
 ******************************************************************************
 * @file           : ano_protocol.c
 * @brief          : ANO Flight Controller Protocol Encoder Implementation
 ******************************************************************************
 * @attention
 *
 * ANO Protocol Implementation
 * Based on ANO protocol specification from 发送示例.txt
 *
 ******************************************************************************
 */

#include "ano_protocol.h"
#include <string.h>

/* Static buffers for DMA transmission (must persist after function returns) */
static uint8_t ano_distance_tx_buf[ANO_HEADER_LEN + ANO_DISTANCE_DATA_LEN +
                                   ANO_CHECKSUM_LEN];
static uint8_t ano_flow_tx_buf[ANO_HEADER_LEN + ANO_FLOW_MODE1_DATA_LEN +
                               ANO_CHECKSUM_LEN];

/**
 * @brief Calculate ANO checksum (SUM and ADD)
 * @param data: Pointer to data buffer (starting from Frame ID)
 * @param length: Length of data (Frame ID + Length + Data bytes)
 * @param sum_check: Pointer to store SUM checksum
 * @param add_check: Pointer to store ADD checksum
 *
 * Algorithm:
 * SUM_CHECK = sum of all bytes
 * ADD_CHECK = cumulative addition (sum of sums)
 */
void ano_calculate_checksum(const uint8_t *data, uint8_t length,
                            uint8_t *sum_check, uint8_t *add_check) {
  uint8_t sum = 0;
  uint8_t add = 0;

  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
    add += sum;
  }

  *sum_check = sum;
  *add_check = add;
}

/**
 * @brief Build ANO frame
 * @param frame_id: Frame ID
 * @param data: Pointer to data payload
 * @param data_len: Length of data payload
 * @param buffer: Output buffer for complete frame
 * @retval Total frame length
 */
static uint8_t ano_build_frame(uint8_t frame_id, const uint8_t *data,
                               uint8_t data_len, uint8_t *buffer) {
  uint8_t idx = 0;

  // Header
  buffer[idx++] = ANO_FRAME_HEADER1; // 0xAA
  buffer[idx++] = ANO_FRAME_HEADER2; // 0xFF
  buffer[idx++] = frame_id;          // Frame ID
  buffer[idx++] = data_len;          // Data length

  // Data payload
  memcpy(&buffer[idx], data, data_len);
  idx += data_len;

  // Calculate checksum (from Frame ID to end of data)
  uint8_t sum_check, add_check;
  ano_calculate_checksum(&buffer[2], 2 + data_len, &sum_check, &add_check);

  // Checksums
  buffer[idx++] = sum_check;
  buffer[idx++] = add_check;

  return idx;
}

/**
 * @brief Send ANO distance frame
 * @param huart: UART handle for transmission
 * @param distance_data: Pointer to distance data structure
 * @retval HAL status
 */
HAL_StatusTypeDef ano_send_distance(UART_HandleTypeDef *huart,
                                    const ano_distance_data_t *distance_data) {
  uint8_t
      frame_buffer[ANO_HEADER_LEN + ANO_DISTANCE_DATA_LEN + ANO_CHECKSUM_LEN];
  uint8_t data_buffer[ANO_DISTANCE_DATA_LEN]; // 7字节

  // 1. 方向 (1字节)
  data_buffer[0] = distance_data->direction;

  // 2. 角度 (2字节, 小端模式)
  data_buffer[1] = (uint8_t)(distance_data->angle & 0xFF);
  data_buffer[2] = (uint8_t)((distance_data->angle >> 8) & 0xFF);

  // 3. 距离 (4字节, 小端模式, 单位CM)
  data_buffer[3] = (uint8_t)(distance_data->distance_cm & 0xFF);
  data_buffer[4] = (uint8_t)((distance_data->distance_cm >> 8) & 0xFF);
  data_buffer[5] = (uint8_t)((distance_data->distance_cm >> 16) & 0xFF);
  data_buffer[6] = (uint8_t)((distance_data->distance_cm >> 24) & 0xFF);

  // 构建并发送帧 (ID 0x34)
  uint8_t frame_len = ano_build_frame(ANO_FRAME_ID_DISTANCE, data_buffer,
                                      ANO_DISTANCE_DATA_LEN, frame_buffer);
  return HAL_UART_Transmit(huart, frame_buffer, frame_len, 100);
}

/**
 * @brief Send ANO optical flow frame (Mode 1)
 * @param huart: UART handle for transmission
 * @param flow_data: Pointer to optical flow data structure
 * @retval HAL status
 */
HAL_StatusTypeDef
ano_send_optical_flow_mode1(UART_HandleTypeDef *huart,
                            const ano_optical_flow_mode1_t *flow_data) {
  uint8_t
      frame_buffer[ANO_HEADER_LEN + ANO_FLOW_MODE1_DATA_LEN + ANO_CHECKSUM_LEN];
  uint8_t data_buffer[ANO_FLOW_MODE1_DATA_LEN];

  // 严格按照手册顺序填充 Buffer
  // [Mode][State][DX_L][DX_H][DY_L][DY_H][Quality]

  data_buffer[0] = flow_data->mode;  // Byte 0
  data_buffer[1] = flow_data->state; // Byte 1 (之前误写为 quality)

  // DX 小端模式
  data_buffer[2] = (uint8_t)(flow_data->dx & 0xFF);
  data_buffer[3] = (uint8_t)((flow_data->dx >> 8) & 0xFF);

  // DY 小端模式
  data_buffer[4] = (uint8_t)(flow_data->dy & 0xFF);
  data_buffer[5] = (uint8_t)((flow_data->dy >> 8) & 0xFF);

  data_buffer[6] = flow_data->quality; // Byte 6 (之前误写为 reserved)

  // 构建并发送帧
  uint8_t frame_len = ano_build_frame(ANO_FRAME_ID_OPTICAL_FLOW, data_buffer,
                                      ANO_FLOW_MODE1_DATA_LEN, frame_buffer);
  return HAL_UART_Transmit(huart, frame_buffer, frame_len, 100);
}
/**
 * @brief Send ANO distance frame via DMA
 * @param huart: UART handle for transmission
 * @param distance_data: Pointer to distance data structure
 * @retval HAL status
 */
HAL_StatusTypeDef
ano_send_distance_dma(UART_HandleTypeDef *huart,
                      const ano_distance_data_t *distance_data) {
  // 使用静态 buffer 以确保 DMA 传输期间数据有效 (ano_distance_tx_buf
  // 已在文件顶部定义) 注意：data_buffer 是临时变量，用于构建
  // payload，最终会被复制到 ano_distance_tx_buf
  uint8_t data_buffer[ANO_DISTANCE_DATA_LEN]; // 长度应为 7

  // 1. 方向 (1字节)
  data_buffer[0] = distance_data->direction;

  // 2. 角度 (2字节, 小端模式)
  data_buffer[1] = (uint8_t)(distance_data->angle & 0xFF);
  data_buffer[2] = (uint8_t)((distance_data->angle >> 8) & 0xFF);

  // 3. 距离 (4字节, 小端模式, 单位CM)
  data_buffer[3] = (uint8_t)(distance_data->distance_cm & 0xFF);
  data_buffer[4] = (uint8_t)((distance_data->distance_cm >> 8) & 0xFF);
  data_buffer[5] = (uint8_t)((distance_data->distance_cm >> 16) & 0xFF);
  data_buffer[6] = (uint8_t)((distance_data->distance_cm >> 24) & 0xFF);

  // 构建完整帧到静态发送缓冲区 (ano_distance_tx_buf)
  // 注意：ano_build_frame 会处理帧头(AA FF)、ID、长度和校验和
  uint8_t frame_len =
      ano_build_frame(ANO_FRAME_ID_DISTANCE, data_buffer, ANO_DISTANCE_DATA_LEN,
                      ano_distance_tx_buf);

  // 通过 DMA 发送静态缓冲区的数据
  return HAL_UART_Transmit_DMA(huart, ano_distance_tx_buf, frame_len);
}

/**
 * @brief Send ANO optical flow frame (Mode 1) via DMA
 * @param huart: UART handle for transmission
 * @param flow_data: Pointer to optical flow data structure
 * @retval HAL status
 */
HAL_StatusTypeDef
ano_send_optical_flow_mode1_dma(UART_HandleTypeDef *huart,
                                const ano_optical_flow_mode1_t *flow_data) {
  // 使用静态 buffer 确保 DMA 安全 (ano_flow_tx_buf 已在文件头部定义)
  uint8_t data_buffer[ANO_FLOW_MODE1_DATA_LEN];

  // 严格按照手册顺序填充: [Mode][State][DX_L][DX_H][DY_L][DY_H][Quality]
  data_buffer[0] = flow_data->mode;  // Byte 0
  data_buffer[1] = flow_data->state; // Byte 1 (状态位)

  // DX (小端模式)
  data_buffer[2] = (uint8_t)(flow_data->dx & 0xFF);
  data_buffer[3] = (uint8_t)((flow_data->dx >> 8) & 0xFF);

  // DY (小端模式)
  data_buffer[4] = (uint8_t)(flow_data->dy & 0xFF);
  data_buffer[5] = (uint8_t)((flow_data->dy >> 8) & 0xFF);

  // Quality (置信度)
  data_buffer[6] = flow_data->quality; // Byte 6

  // 构建完整帧到静态发送缓冲区
  uint8_t frame_len = ano_build_frame(ANO_FRAME_ID_OPTICAL_FLOW, data_buffer,
                                      ANO_FLOW_MODE1_DATA_LEN, ano_flow_tx_buf);

  // Transmit via DMA (non-blocking)
  return HAL_UART_Transmit_DMA(huart, ano_flow_tx_buf, frame_len);
}
