#!/bin/bash

# --- 配置部分 ---
# 1. 指定 OpenOCD 配置文件
OPENOCD_CFG="openocd_dap.cfg"

# 2. 指定固件路径
# 如果你是用 CMake 编译的，默认生成在 build 目录下，文件名通常是 Money (ELF格式)
FIRMWARE_PATH="build/Money.elf"

# 如果你想烧录 Keil/MDK 生成的 hex 文件，请取消下面这行的注释，并注释掉上面那行
# FIRMWARE_PATH="MDK-ARM/Money/Money.hex"
# ----------------

# 检查固件文件是否存在
if [ ! -f "$FIRMWARE_PATH" ]; then
    echo "错误: 找不到固件文件 -> $FIRMWARE_PATH"
    echo "请先执行编译 (例如: cmake --build build)"
    exit 1
fi

echo "正在使用 $OPENOCD_CFG 烧录 $FIRMWARE_PATH ..."

# 调用 OpenOCD 执行烧录
# program: 烧录命令
# verify: 烧录后校验
# reset: 烧录后复位芯片
# exit: 完成后退出 OpenOCD
openocd -f "$OPENOCD_CFG" -c "program $FIRMWARE_PATH verify reset exit"

if [ $? -eq 0 ]; then
    echo "✅ 烧录成功！"
else
    echo "❌ 烧录失败，请检查连接。"
fi