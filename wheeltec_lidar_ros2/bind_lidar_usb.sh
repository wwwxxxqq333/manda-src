#!/bin/bash
# ============================================================
# 绑定 LSLIDAR X10 雷达 USB 串口到 /dev/lidar_usb
# 使用方法: sudo bash bind_lidar_usb.sh
# ============================================================

RULES_FILE="/etc/udev/rules.d/lidar_usb.rules"

echo "正在检测雷达 USB 设备..."
# 查找 CP210x (雷达也是 CP210x 芯片) 或 CH340/FTDI
LIDAR_INFO=$(lsusb | grep -iE "cp210|ch340|ch341|ftdi|pl2303|cp210x" | grep -v "Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001")

if [ -z "$LIDAR_INFO" ]; then
    echo "❌ 未检测到第二个 USB 串口设备"
    echo "   当前 USB 串口设备:"
    lsusb | grep -iE "cp210|ch340|ch341|ftdi|pl2303"
    echo ""
    echo "💡 请确保雷达 USB 线已连接，然后重新运行此脚本"
    exit 1
fi

echo "检测到设备: $LIDAR_INFO"

# 获取第一个非 IMU 的 CP210x 设备
VENDOR=$(echo "$LIDAR_INFO" | head -1 | awk '{print $6}' | cut -d: -f1)
PRODUCT=$(echo "$LIDAR_INFO" | head -1 | awk '{print $6}' | cut -d: -f2)

if [ -z "$VENDOR" ] || [ -z "$PRODUCT" ]; then
    # 回退：匹配所有 ttyUSB 设备并排除 /dev/imu_usb 指向的那个
    echo "使用通用匹配规则..."
    cat > "$RULES_FILE" << 'UDEVEOF'
# LSLIDAR X10 雷达 USB 串口绑定
# 匹配第二个 ttyUSB 设备（排除 IMU 占用的 CP2102_0001）
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{serial}=="0001", SYMLINK+="imu_usb"
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{serial}!="0001", SYMLINK+="lidar_usb"
UDEVEOF
else
    echo "写入 udev 规则: vendor=$VENDOR, product=$PRODUCT"
    cat > "$RULES_FILE" << UDEVEOF
# LSLIDAR X10 雷达 USB 串口绑定
SUBSYSTEM=="tty", ATTRS{idVendor}=="$VENDOR", ATTRS{idProduct}=="$PRODUCT", SYMLINK+="lidar_usb", MODE="0666"
UDEVEOF
fi

echo "规则已写入: $RULES_FILE"
echo "重新加载 udev..."
udevadm control --reload-rules
udevadm trigger

echo ""
echo "✅ 完成！请拔插雷达 USB 线，然后执行:"
echo "   ls -la /dev/lidar_usb"
