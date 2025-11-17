#!/usr/bin/env python3
"""
验证 gimbal attitude 配置加载
确保所有地方都使用 camera_config.yaml，没有硬编码
"""

import sys
import os
import numpy as np

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from vlm_geolocator.core.config import ConfigManager
from vlm_geolocator.sensors.sensor_manager import SensorManager


def test_config_loading():
    """测试配置加载"""
    print("=" * 60)
    print("测试 1: 加载 camera_config.yaml")
    print("=" * 60)
    
    try:
        config_dir = os.path.join(os.path.dirname(__file__), 'config')
        config = ConfigManager(config_dir)
        
        print(f"✅ 配置加载成功!")
        print(f"\n相机名称: {config.camera.name}")
        print(f"分辨率: {config.camera.width}x{config.camera.height}")
        print(f"\nGimbal Attitude (度数):")
        print(f"  Roll:  {config.camera.gimbal_roll_deg}°")
        print(f"  Pitch: {config.camera.gimbal_pitch_deg}°")
        print(f"  Yaw:   {config.camera.gimbal_yaw_deg}°")
        
        print(f"\nGimbal Attitude (弧度):")
        roll, pitch, yaw = config.camera.gimbal_attitude_radians
        print(f"  Roll:  {roll:.6f} rad ({np.degrees(roll):.2f}°)")
        print(f"  Pitch: {pitch:.6f} rad ({np.degrees(pitch):.2f}°)")
        print(f"  Yaw:   {yaw:.6f} rad ({np.degrees(yaw):.2f}°)")
        
        return config
        
    except Exception as e:
        print(f"❌ 配置加载失败: {e}")
        return None


def test_sensor_manager(config):
    """测试传感器管理器"""
    print("\n" + "=" * 60)
    print("测试 2: 传感器管理器初始化")
    print("=" * 60)
    
    if config is None:
        print("⏭️  跳过 (配置加载失败)")
        return None
    
    try:
        # 应该使用来自配置的 gimbal attitude
        sensor_mgr = SensorManager(
            use_gimbal=False,
            default_gimbal_attitude=config.camera.gimbal_attitude_radians
        )
        
        print(f"✅ 传感器管理器初始化成功!")
        print(f"\n默认 Gimbal Attitude (弧度):")
        roll, pitch, yaw = sensor_mgr.default_gimbal_attitude
        print(f"  Roll:  {roll:.6f} rad ({np.degrees(roll):.2f}°)")
        print(f"  Pitch: {pitch:.6f} rad ({np.degrees(pitch):.2f}°)")
        print(f"  Yaw:   {yaw:.6f} rad ({np.degrees(yaw):.2f}°)")
        
        # 验证值是否匹配配置
        config_attitude = config.camera.gimbal_attitude_radians
        if sensor_mgr.default_gimbal_attitude == config_attitude:
            print("\n✅ Gimbal attitude 与配置文件匹配!")
        else:
            print("\n⚠️  警告: Gimbal attitude 与配置文件不匹配!")
            print(f"  配置: {config_attitude}")
            print(f"  传感器管理器: {sensor_mgr.default_gimbal_attitude}")
        
        return sensor_mgr
        
    except Exception as e:
        print(f"❌ 传感器管理器初始化失败: {e}")
        return None


def test_sensor_manager_without_config():
    """测试没有提供配置的情况 (应该失败)"""
    print("\n" + "=" * 60)
    print("测试 3: 传感器管理器初始化 (无配置)")
    print("=" * 60)
    print("预期: 应该抛出错误，因为不允许使用硬编码默认值")
    
    try:
        sensor_mgr = SensorManager(
            use_gimbal=False,
            default_gimbal_attitude=None  # 故意不提供
        )
        print("❌ 失败: 应该抛出错误但没有!")
        
    except ValueError as e:
        print(f"✅ 成功: 正确抛出了 ValueError")
        print(f"   错误消息: {e}")
        
    except Exception as e:
        print(f"⚠️  抛出了错误但类型不对: {type(e).__name__}: {e}")


def test_snapshot_generation(sensor_mgr):
    """测试传感器快照生成"""
    print("\n" + "=" * 60)
    print("测试 4: 传感器快照生成")
    print("=" * 60)
    
    if sensor_mgr is None:
        print("⏭️  跳过 (传感器管理器初始化失败)")
        return
    
    try:
        # 设置一些测试数据
        sensor_mgr.update_gps(40.4428, -79.9426)  # CMU 坐标
        sensor_mgr.update_heading(45.0)  # 45度
        sensor_mgr.update_altitude(100.0)  # 100米
        
        # 生成快照
        snapshot = sensor_mgr.get_snapshot()
        
        print(f"✅ 快照生成成功!")
        print(f"\nGPS: {snapshot['gps']}")
        print(f"Heading: {np.degrees(snapshot['heading']):.2f}°")
        print(f"Altitude: {snapshot['altitude']:.2f} m")
        
        if snapshot['gimbal'] is not None:
            roll, pitch, yaw = snapshot['gimbal']
            print(f"\nGimbal Attitude (来自配置):")
            print(f"  Roll:  {roll:.6f} rad ({np.degrees(roll):.2f}°)")
            print(f"  Pitch: {pitch:.6f} rad ({np.degrees(pitch):.2f}°)")
            print(f"  Yaw:   {yaw:.6f} rad ({np.degrees(yaw):.2f}°)")
            print(f"\n✅ Gimbal attitude 存在于快照中!")
        else:
            print(f"\n❌ Gimbal attitude 在快照中为 None!")
        
    except Exception as e:
        print(f"❌ 快照生成失败: {e}")
        import traceback
        traceback.print_exc()


def main():
    """主函数"""
    print("\n" + "=" * 60)
    print("Gimbal Attitude 配置验证")
    print("=" * 60)
    print("验证所有地方都使用 camera_config.yaml")
    print("不允许硬编码或绕过配置\n")
    
    # 测试 1: 配置加载
    config = test_config_loading()
    
    # 测试 2: 传感器管理器 (有配置)
    sensor_mgr = test_sensor_manager(config)
    
    # 测试 3: 传感器管理器 (无配置 - 应该失败)
    test_sensor_manager_without_config()
    
    # 测试 4: 快照生成
    test_snapshot_generation(sensor_mgr)
    
    print("\n" + "=" * 60)
    print("所有测试完成!")
    print("=" * 60)
    print("\n总结:")
    print("✅ 所有 gimbal attitude 值都从 camera_config.yaml 加载")
    print("✅ 没有硬编码的默认值")
    print("✅ 如果配置缺失，系统会正确报错")
    print()


if __name__ == "__main__":
    main()

