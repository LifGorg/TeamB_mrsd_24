#!/usr/bin/env python3
"""
测试脚本：验证RTSP流媒体节点的硬件加速功能
"""

import subprocess
import time
import psutil
import os
import signal
import sys

def get_cpu_usage():
    """获取当前CPU使用率"""
    return psutil.cpu_percent(interval=1)

def get_gpu_usage():
    """获取GPU使用率（如果可用）"""
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            return float(result.stdout.strip())
    except:
        pass
    return None

def test_rtsp_node():
    """测试RTSP节点性能"""
    print("🚀 启动RTSP流媒体节点硬件加速测试...")
    
    # 记录初始CPU使用率
    initial_cpu = get_cpu_usage()
    print(f"📊 初始CPU使用率: {initial_cpu:.1f}%")
    
    initial_gpu = get_gpu_usage()
    if initial_gpu is not None:
        print(f"📊 初始GPU使用率: {initial_gpu:.1f}%")
    
    # 启动ROS2节点
    print("\n🔄 启动ROS2 RTSP流媒体节点...")
    
    # 进入Docker容器并运行节点
    docker_cmd = [
        'docker', 'exec', '-it', 'airstack-unified',
        'bash', '-c', 
        'source /root/ros_ws/install/setup.bash && '
        'ros2 run rtsp_streamer rtsp_streamer_node_gstreamer --ros-args '
        '-p rtsp_url:=rtsp://10.3.1.124:8556/ghadron '
        '-p fps:=2.0 '
        '-p width:=640 '
        '-p height:=512'
    ]
    
    try:
        # 启动节点进程
        process = subprocess.Popen(docker_cmd, 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.STDOUT,
                                 universal_newlines=True,
                                 bufsize=1)
        
        print("⏳ 等待节点启动...")
        time.sleep(5)
        
        # 监控性能10秒
        print("\n📈 监控性能指标 (10秒)...")
        cpu_readings = []
        gpu_readings = []
        
        for i in range(10):
            cpu = get_cpu_usage()
            gpu = get_gpu_usage()
            
            cpu_readings.append(cpu)
            if gpu is not None:
                gpu_readings.append(gpu)
            
            print(f"⏱️  {i+1:2d}s - CPU: {cpu:5.1f}%", end="")
            if gpu is not None:
                print(f", GPU: {gpu:5.1f}%")
            else:
                print()
            
            # 检查进程输出
            if process.poll() is not None:
                print("❌ 节点进程已退出")
                break
                
        # 计算平均值
        avg_cpu = sum(cpu_readings) / len(cpu_readings) if cpu_readings else 0
        avg_gpu = sum(gpu_readings) / len(gpu_readings) if gpu_readings else None
        
        print(f"\n📊 性能统计:")
        print(f"   平均CPU使用率: {avg_cpu:.1f}%")
        if avg_gpu is not None:
            print(f"   平均GPU使用率: {avg_gpu:.1f}%")
        
        # 分析结果
        print(f"\n🔍 分析结果:")
        if avg_cpu < 30:
            print("   ✅ CPU使用率正常 - 可能使用了硬件加速")
        elif avg_cpu < 60:
            print("   ⚠️  CPU使用率中等 - 可能部分使用硬件加速")
        else:
            print("   ❌ CPU使用率过高 - 可能使用软件解码")
            
        if avg_gpu is not None and avg_gpu > 5:
            print("   ✅ 检测到GPU活动 - 硬件加速可能正在工作")
        elif avg_gpu is not None:
            print("   ⚠️  GPU使用率较低 - 检查硬件加速配置")
        
    except KeyboardInterrupt:
        print("\n⏹️  测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试过程中出错: {e}")
    finally:
        # 清理进程
        try:
            if 'process' in locals():
                process.terminate()
                process.wait(timeout=5)
        except:
            try:
                process.kill()
            except:
                pass
        
        print("\n🧹 清理完成")

def check_docker_container():
    """检查Docker容器是否运行"""
    try:
        result = subprocess.run(['docker', 'ps', '--filter', 'name=airstack-unified', '--format', '{{.Names}}'], 
                              capture_output=True, text=True)
        if 'airstack-unified' in result.stdout:
            print("✅ Docker容器 'airstack-unified' 正在运行")
            return True
        else:
            print("❌ Docker容器 'airstack-unified' 未运行")
            print("   请先启动容器: docker run ... airstack-unified")
            return False
    except Exception as e:
        print(f"❌ 检查Docker容器时出错: {e}")
        return False

def main():
    print("🔧 RTSP流媒体节点硬件加速测试工具")
    print("=" * 50)
    
    # 检查依赖
    if not check_docker_container():
        return 1
    
    # 运行测试
    test_rtsp_node()
    
    print("\n✨ 测试完成！")
    return 0

if __name__ == "__main__":
    sys.exit(main())

