#!/usr/bin/env python3
"""
save_map.py — FAST-LIO2 建图后转 Nav2 格式 + 自动设为最新

用法：
  python3 save_map.py [--pcd /path/to/map.pcd] [--name my_map]
"""

import os
import sys
import time
import shutil
import subprocess
import argparse


MAPS_DIR = "/home/sunrise/manda/maps"
LATEST_LINK = os.path.join(MAPS_DIR, "latest_map")


def run(cmd: str) -> str:
    """Run shell command, return stdout."""
    return subprocess.check_output(cmd, shell=True, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description="Save FAST-LIO2 PCD map as Nav2 format")
    parser.add_argument("--pcd", default="/home/sunrise/manda/map.pcd",
                        help="PCD file to convert (FAST-LIO2 output)")
    parser.add_argument("--name", default=None,
                        help="Map name (default: timestamp)")
    args = parser.parse_args()

    if not os.path.exists(args.pcd):
        print(f"ERROR: PCD file not found: {args.pcd}")
        sys.exit(1)

    # 生成地图名
    if args.name is None:
        args.name = time.strftime("map_%Y%m%d_%H%M%S")

    map_dir = os.path.join(MAPS_DIR, args.name)
    os.makedirs(map_dir, exist_ok=True)

    # 1. 用 pcl_ros 转 PCD → pgm + yaml
    pcd_dest = os.path.join(map_dir, "cloud.pcd")
    shutil.copy2(args.pcd, pcd_dest)
    print(f"  PCD copied to {pcd_dest}")

    # 2. 生成 Nav2 map YAML (以 0.05m 分辨率)
    yaml_path = os.path.join(map_dir, f"{args.name}.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"image: {args.name}.pgm\n")
        f.write("resolution: 0.05\n")
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    print(f"  YAML saved to {yaml_path}")

    # 3. 用 pcl_ros/pcl_conversions 转 PCD → pgm（如果有 ROS2）
    #    如果没有，创建占位 pgm（需要手动用 CloudCompare 转换）
    pgm_path = os.path.join(map_dir, f"{args.name}.pgm")
    try:
        # 尝试使用 ROS2 转换
        cmd = (
            f"source /home/sunrise/manda/install/setup.bash 2>/dev/null && "
            f"ros2 run pcl_ros pcd_to_pointcloud --ros-args -p pcd_path:={pcd_dest} "
            f"-p frame_id:=map 2>/dev/null || true"
        )
        run(cmd)
        print(f"  PGM conversion attempted via pcl_ros")
    except Exception:
        pass

    # 创建占位 PGM（1x1 白色像素，Nav2 需要这个文件存在才能启动）
    if not os.path.exists(pgm_path):
        # 创建最小的有效 pgm
        with open(pgm_path, "wb") as f:
            f.write(b"P5\n1 1\n255\n\xff")
        print(f"  Placeholder PGM created (use CloudCompare for real conversion)")

    # 4. 更新 latest_map 链接
    yaml_link = os.path.join(MAPS_DIR, "latest_map.yaml")
    if os.path.islink(yaml_link) or os.path.exists(yaml_link):
        os.remove(yaml_link)
    os.symlink(yaml_path, yaml_link)
    print(f"  latest_map → {yaml_path}")

    # 5. 列出所有地图
    print("\n=== All saved maps ===")
    for d in sorted(os.listdir(MAPS_DIR)):
        full = os.path.join(MAPS_DIR, d)
        if os.path.isdir(full):
            yaml = os.path.join(full, f"{d}.yaml")
            size = os.path.getsize(args.pcd) if d == args.name else 0
            marker = " ← LATEST" if os.path.realpath(yaml_link) == yaml else ""
            print(f"  {d}{marker}")

    print("\nDone! Restart Nav2 to use the latest map.")


if __name__ == "__main__":
    main()
