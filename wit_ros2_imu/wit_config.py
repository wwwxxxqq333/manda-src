#!/usr/bin/env python3
"""
WitMotion IMU 串口配置工具 — Linux 版
用于将 IMU 配置为：波特率 115200 + 标准协议 (0x55 头)
"""

import serial
import time
import sys

DEVICE = '/dev/imu_usb'

# 波特率 ID: 0x01=4800, 0x06=115200, 0x08=460800
def send_cmd(ser, cmd_bytes, desc):
    """发送命令并等待响应"""
    ser.reset_input_buffer()
    ser.write(bytes(cmd_bytes))
    time.sleep(0.3)
    resp = ser.read(20)
    print(f'  [{desc}] 发送: {" ".join([f"{b:02X}" for b in cmd_bytes])}')
    if resp:
        print(f'          响应: {" ".join([f"{b:02X}" for b in resp])}')
    return resp

def try_protocol_switch(baud=460800):
    """尝试多种协议切换命令"""
    print(f'\n  尝试以 {baud} baud 发送协议切换命令...')
    try:
        ser = serial.Serial(DEVICE, baudrate=baud, timeout=0.5)
        time.sleep(0.5)
        ser.reset_input_buffer()
        _ = ser.read(20)  # 清缓冲

        # 方法1: 寄存器 0x02 — 设置 RS232 输出内容
        # 启用: 加速度(bit1)+角速度(bit2)+角度(bit3)+磁场(bit4)+四元数(bit8)
        # = 0b1_0001_1110 = 0x011E, 低字节在前
        cmd1 = [0xFF, 0xAA, 0x02, 0x02, 0x00, 0x1E, 0x01]
        send_cmd(ser, cmd1, '输出内容(加速度+角速度+角度+磁场+四元数)')

        # 方法2: 寄存器 0x1C — 协议类型 (0=标准0x55, 1=JY901)
        cmd2 = [0xFF, 0xAA, 0x1C, 0x01, 0x00, 0x00]
        send_cmd(ser, cmd2, '协议类型→标准(0x55)')

        # 方法3: 寄存器 0x1A — 另一种协议切换方式
        cmd3 = [0xFF, 0xAA, 0x1A, 0x01, 0x00, 0x00]
        send_cmd(ser, cmd3, '寄存器0x1A→0')

        # 保存
        cmd_save = [0xFF, 0xAA, 0x00, 0x00, 0x00]
        send_cmd(ser, cmd_save, '保存到Flash')
        
        ser.close()
        return True
    except Exception as e:
        print(f'  协议切换失败: {e}')
        return False

def verify(baud=115200):
    """验证配置结果"""
    print(f'\n  以 {baud} baud 验证...')
    try:
        ser = serial.Serial(DEVICE, baudrate=baud, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        data = ser.read(300)
        ser.close()
        print(f'  收到 {len(data)} 字节')

        p55 = [i for i, b in enumerate(data) if b == 0x55]
        pA5 = [i for i, b in enumerate(data) if b == 0xA5]

        if p55:
            print(f'\n  ✅✅✅ 成功！检测到 0x55 标准协议头！')
            print(f'  0x55 出现位置: {p55[:10]}')
            print(f'  数据样本: {" ".join([f"{b:02X}" for b in data[:40]])}')
            return True
        else:
            print(f'  数据样本: {" ".join([f"{b:02X}" for b in data[:40]])}')
            if pA5:
                print(f'  ⚠ 仍为 JY901 协议 (A5 5A)')
            else:
                print(f'  ⚠ 未检测到已知协议头')
            return False
    except Exception as e:
        print(f'  验证失败: {e}')
        return False

def main():
    print('=' * 55)
    print('  WitMotion IMU 配置工具 v2')
    print('  目标: 115200 baud + 标准协议(0x55)')
    print('=' * 55)

    # === 阶段1: 先切换协议 (在460800下操作) ===
    print('\n[阶段1] 在当前 460800 波特率下切换协议...')
    try_protocol_switch(460800)

    # === 阶段2: 设置波特率 ===
    print('\n[阶段2] 设置波特率为 115200...')
    try:
        ser = serial.Serial(DEVICE, baudrate=460800, timeout=0.5)
        time.sleep(0.5)
        ser.reset_input_buffer()
        _ = ser.read(20)
        cmd_baud = [0xFF, 0xAA, 0x04, 0x01, 0x00, 0x06]  # 0x06=115200
        send_cmd(ser, cmd_baud, '设置波特率115200')
        cmd_save = [0xFF, 0xAA, 0x00, 0x00, 0x00]
        send_cmd(ser, cmd_save, '保存设置')
        ser.close()
        time.sleep(1)
    except Exception as e:
        print(f'  波特率设置失败: {e}')

    # === 阶段3: 验证 ===
    print('\n[阶段3] 验证...')
    if verify(115200):
        print('\n🎉 配置成功！可以启动 ROS 驱动了。')
    else:
        print('\n[阶段4] 再尝试一次协议切换 (在115200下)...')
        try_protocol_switch(115200)
        time.sleep(1)
        if verify(115200):
            print('\n🎉 配置成功！')
        else:
            print('\n❌ 自动配置未能完全成功。')
            print('  请检查 IMU 型号，可能需要在 Windows 上位机中手动配置。')
            print('  或者将驱动代码改为适配 JY901 协议 (A5 5A)。')


if __name__ == '__main__':
    main()
