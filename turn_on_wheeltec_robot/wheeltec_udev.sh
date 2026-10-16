#!/bin/bash

echo "开始配置 manda_robot 专属 udev 规则..."

# === 1. IMU (WitMotion) 规则绑定 ===
# CP2102 串口号0002 设置别名为 imu_usb (原厂写的是底盘，现已强行写死为 IMU)
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="imu_usb"' >/etc/udev/rules.d/imu_usb.rules

# === 2. 激光雷达 (LSLIDAR) 规则绑定 ===
# CP2102 串口号0001 设置别名为 wheeltec_lidar
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules

# === 3. 其他保留规则 (GNSS、MIC、相机等，以备后用) ===
# GNSS
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gnss"' >/etc/udev/rules.d/wheeltec_gnss.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_gnss"' >/etc/udev/rules.d/wheeltec_gps.rules

# # 相机 (Video4Linux)
# echo 'SUBSYSTEM=="video4linux",ATTR{name}=="GENERAL WEBCAM",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >/etc/udev/rules.d/camera.rules
# echo 'SUBSYSTEM=="video4linux",ATTR{name}=="Astra Pro HD Camera: Astra Pro ",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Pro"' >>/etc/udev/rules.d/camera.rules
# echo 'SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="realsense"' >>/etc/udev/rules.d/camera.rules

# === 4. 清理可能残留的冲突规则 ===
# 防止原厂的 0002 底盘规则残留在系统中干扰 IMU
rm -f /etc/udev/rules.d/wheeltec_controller.rules
rm -f /etc/udev/rules.d/wheeltec_controller2.rules
rm -f /etc/udev/rules.d/wheeltec_controller3.rules
rm -f /etc/udev/rules.d/99-manda-sensors.rules
rm -f /etc/udev/rules.d/99-manda-custom.rules

# === 5. 重启服务使之生效 ===
service udev reload
sleep 2
service udev restart
echo "规则配置完成并已生效！"


