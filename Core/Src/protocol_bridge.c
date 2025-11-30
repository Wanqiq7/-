#include "protocol_bridge.h"
#include <math.h>
#include <string.h>

// 滤波系数
#define FLOW_LPF_ALPHA 0.85f
// 在 FLOW_LPF_ALPHA 附近添加
#define DISTANCE_LPF_ALPHA 0.90f // 高度滤波系数 (0.0-1.0), 越大越平滑但延迟越高

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

  // 静态变量保存历史值用于滤波
  static float last_vel_fc_x = 0.0f;
  static float last_vel_fc_y = 0.0f;

  // 【新增】静态变量保存上一次的高度值
  static float last_height_cm = 0.0f;

  static bool is_first_run = true;

  // 1. 设置模式
  ano_flow->mode = 1;
  // 2. 设置状态
  ano_flow->state = (mavlink_flow->quality > 50) ? 1 : 0;

  // 3. 计算原始物理速度
  float raw_vel_fc_x = 0.0f;
  float raw_vel_fc_y = 0.0f;

  // 【新增】高度滤波逻辑
  // 如果是第一次运行，直接赋值；否则执行低通滤波
  if (is_first_run) {
    last_height_cm = distance_cm;
  } else {
    // 只有当传入的高度有效时(>0)才更新滤波，防止传入0把高度拉低
    if (distance_cm > 0.1f) {
      last_height_cm =
          last_height_cm + DISTANCE_LPF_ALPHA * (distance_cm - last_height_cm);
    }
  }
  // 【安全保护】防止高度衰减到0导致计算出错
  if (last_height_cm < 1.0f)
    last_height_cm = 1.0f;

  if (mavlink_flow->integration_time_us > 0) {
    float dt_s = (float)mavlink_flow->integration_time_us / 1000000.0f;

    // ============================================================
    // 【修改】: 使用滤波后的高度 (last_height_cm) 参与计算
    // ============================================================

    // 1. 飞控 X 轴 (机头) 速度
    // raw_vel_fc_x = (mavlink_flow->integrated_y / dt_s) * distance_cm; //
    // 原代码
    raw_vel_fc_x =
        (mavlink_flow->integrated_y / dt_s) * last_height_cm; // 修改后

    // 2. 飞控 Y 轴 (左侧) 速度
    // raw_vel_fc_y = (mavlink_flow->integrated_x / dt_s) * distance_cm; //
    // 原代码
    raw_vel_fc_y =
        (mavlink_flow->integrated_x / dt_s) * last_height_cm; // 修改后
  }

  // 4. 执行速度低通滤波
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

  // 5. 填充数据
  ano_flow->dx = constrain_int16(last_vel_fc_x);
  ano_flow->dy = constrain_int16(last_vel_fc_y);
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
