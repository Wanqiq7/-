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
 * - MAVLINK integrated_x/y are in radians (accumulated flow over integration
 * time)
 * - ANO expects flow velocity as int16 values
 * - We scale radians by FLOW_RAD_TO_ANO_SCALE (1000.0) to get reasonable int16
 * values
 * - Quality maps directly (0-255)
 */
bool bridge_convert_optical_flow(
    const mavlink_optical_flow_rad_t *mavlink_flow,
    float distance_cm, // 注意：这里需要传入高度用于计算速度
    ano_optical_flow_mode1_t *ano_flow) {
  if (mavlink_flow == NULL || ano_flow == NULL) {
    return false;
  }

  // 1. 设置模式
  ano_flow->mode = 1; // Mode 1: 融合数据

  // 2. 设置状态 (State)
  // 根据置信度判断数据是否有效，阈值可设为 50 或 100
  ano_flow->state = (mavlink_flow->quality > 50) ? 1 : 0;

  // 3. 计算地面速度 (cm/s)
  // 公式: V = (rad / time_s) * dist_cm
  if (mavlink_flow->integration_time_us > 0) {
    float dt_s = (float)mavlink_flow->integration_time_us / 1000000.0f;

    // 限制最小有效高度以防止噪声放大，例如 10cm
    float valid_dist = (distance_cm > 10.0f) ? distance_cm : 100.0f;

    // 计算速度
    float vel_x = (mavlink_flow->integrated_x / dt_s) * valid_dist;
    float vel_y = (mavlink_flow->integrated_y / dt_s) * valid_dist;

    // 4. 填充速度数据 (注意坐标系修正)
    // 优象T2(右正) -> 匿名(左正)，Y轴取反
    ano_flow->dx = (int16_t)vel_x;
    ano_flow->dy = (int16_t)(-vel_y);
  } else {
    ano_flow->dx = 0;
    ano_flow->dy = 0;
  }

  // 5. 设置质量 (Quality)
  // 放在最后一个字节
  ano_flow->quality = mavlink_flow->quality;

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
                             ano_distance_data_t *ano_distance) {
  if (mavlink_distance == NULL || ano_distance == NULL) {
    return false;
  }
  memset(ano_distance, 0, sizeof(ano_distance_data_t));

  // ANO ID 0x34 需要 CM，直接赋值
  ano_distance->distance_cm = (uint32_t)mavlink_distance->current_distance;

  // 固定方向向下
  ano_distance->direction = 0;
  ano_distance->angle = 0;

  return true;
}
