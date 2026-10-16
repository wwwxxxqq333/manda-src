#!/usr/bin/env python3
"""WitMotion IMU 配置: 9600 baud + 标准协议(0x55)"""
import serial, time, sys

DEV = '/dev/imu_usb'

def send(ser, cmd, desc):
    ser.reset_input_buffer()
    ser.write(bytes(cmd))
    time.sleep(0.3)
    resp = ser.read(20)
    print(f'  [{desc}] 发送: {" ".join([f"{b:02X}" for b in cmd])}')
    if resp:
        print(f'          响应: {" ".join([f"{b:02X}" for b in resp])}')

def main():
    print('=' * 50)
    print('  IMU 配置: 9600 baud + 标准协议(0x55)')
    print('=' * 50)

    # === 阶段1: 460800 连接, 切换协议 + 改波特率 ===
    print('\n[1] 以 460800 baud 连接...')
    ser = serial.Serial(DEV, baudrate=460800, timeout=0.5)
    time.sleep(0.5)
    ser.read(50)  # 清缓冲
    print('  已连接')

    # 输出内容: 加速度+角速度+角度+磁场+四元数 (bit1,2,3,4,8)
    print('\n[2] 设置输出内容...')
    send(ser, [0xFF, 0xAA, 0x02, 0x02, 0x00, 0x1E, 0x01], '输出内容')

    # 协议切换: 寄存器0x1C = 0 (标准协议)
    print('\n[3] 切换协议为标准...')
    send(ser, [0xFF, 0xAA, 0x1C, 0x01, 0x00, 0x00], '协议→标准')

    # 备用协议切换
    send(ser, [0xFF, 0xAA, 0x1A, 0x01, 0x00, 0x00], '寄存器0x1A→0')

    # 改波特率: 9600 = 0x02
    print('\n[4] 设置波特率为 9600...')
    send(ser, [0xFF, 0xAA, 0x04, 0x01, 0x00, 0x02], '波特率→9600')

    # 保存
    print('\n[5] 保存到 Flash...')
    send(ser, [0xFF, 0xAA, 0x00, 0x00, 0x00], '保存Flash')
    ser.close()

    # === 阶段2: 9600 验证 ===
    print('\n[6] 以 9600 baud 验证...')
    time.sleep(1)
    ser2 = serial.Serial(DEV, baudrate=9600, timeout=2)
    time.sleep(0.5)
    ser2.reset_input_buffer()
    data = ser2.read(300)
    ser2.close()
    print(f'  收到 {len(data)} 字节')
    
    p55 = [i for i,b in enumerate(data) if b==0x55]
    if p55:
        print(f'\n  ✅✅✅ 成功！0x55 在位置 {p55[:8]}')
        print(f'  数据: {" ".join([f"{b:02X}" for b in data[:50]])}')
    else:
        print(f'  数据: {" ".join([f"{b:02X}" for b in data[:50]])}')
        print('\n  ⚠ 未检测到 0x55，将同时修改驱动代码适配')

if __name__ == '__main__':
    main()
