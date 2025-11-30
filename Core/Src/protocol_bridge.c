#include "protocol_bridge.h"
#include <math.h>
#include <string.h>

// 滤波系数
#define FLOW_LPF_ALPHA 0.80f

// 限幅辅助函数
static int16_t constrain_int16(float val) {
  if (val > 32000.0f)
    return 32000;
  if (val < -32000.0f)
    return -32000;
  return (int16_t)val;
}

bool bridge_convert_optical_flow(const mavlink_optical_flow_rad_t *mavlink_flow,
                                 float distance_cm,
                                 ano_optical_flow_mode1_t *ano_flow) {

  if (mavlink_flow == NULL || ano_flow == NULL) {
    return false;
  }

  // 静态变量保存历史值用于滤波 (对应飞控的 X 和 Y)
  static float last_vel_fc_x = 0.0f; // 飞控 X 轴速度 (机头)
  static float last_vel_fc_y = 0.0f; // 飞控 Y 轴速度 (左侧)
  static bool is_first_run = true;

  // 1. 设置模式
  ano_flow->mode = 1;
  // 2. 设置状态
  ano_flow->state = (mavlink_flow->quality > 50) ? 1 : 0;

  // 3. 计算原始物理速度
  float raw_vel_fc_x = 0.0f;
  float raw_vel_fc_y = 0.0f;

  if (mavlink_flow->integration_time_us > 0) {
    float dt_s = (float)mavlink_flow->integration_time_us / 1000000.0f;

    // ============================================================
    // 【核心修改】: 光流数据 -> 飞控坐标系映射
    // ============================================================

    // 1. 飞控 X 轴 (机头) 速度
    // 数据源: 光流 Y (integrated_y)
    // 逻辑: integrated_y 代表纵向运动，无需取反
    raw_vel_fc_x = (mavlink_flow->integrated_y / dt_s) * distance_cm;

    // 2. 飞控 Y 轴 (左侧) 速度
    // 数据源: 光流 X (integrated_x)
    // 逻辑: integrated_x 代表横向运动，且左移产生正值，无需取反
    raw_vel_fc_y = (mavlink_flow->integrated_x / dt_s) * distance_cm;
  }

  // 4. 执行低通滤波
  if (is_first_run) {
    last_vel_fc_x = raw_vel_fc_x;
    last_vel_fc_y = raw_vel_fc_y;
    is_first_run = false;
  } else {
    if (ano_flow->state == 1) {
      last_vel_fc_x =
          last_vel_fc_x + FLOW_LPF_ALPHA * (raw_vel_fc_x - last_vel_fc_x);
      last_vel_fc_y =
          last_vel_fc_y + FLOW_LPF_ALPHA * (raw_vel_fc_y - last_vel_fc_y);
    } else {
      last_vel_fc_x *= 0.8f;
      last_vel_fc_y *= 0.8f;
    }
  }

  // 5. 填充数据 (直接对应，无需再交换或取负)
  ano_flow->dx = constrain_int16(last_vel_fc_x); // 对应飞控 X
  ano_flow->dy = constrain_int16(last_vel_fc_y); // 对应飞控 Y

  // 6. 设置质量
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
