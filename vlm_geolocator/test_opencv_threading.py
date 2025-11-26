#!/usr/bin/env python3
"""测试OpenCV在多线程环境中的显示问题"""
import cv2
import numpy as np
import threading
import time
from gi.repository import GLib
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

print("=" * 60)
print("OpenCV Threading Test")
print("=" * 60)

# 测试1: 从主线程显示
def test_main_thread_display():
    print("\n[Test 1] Displaying from main thread...")
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    cv2.putText(frame, "Main Thread", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Test - Main Thread", frame)
    print("  cv2.imshow() called, waiting for key...")
    cv2.waitKey(2000)  # 显示2秒
    cv2.destroyAllWindows()
    print("  [OK] Main thread display works")

# 测试2: 从子线程显示
def display_from_thread():
    print("  [Thread] Attempting to display...")
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    cv2.putText(frame, "Child Thread", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    try:
        cv2.imshow("Test - Child Thread", frame)
        cv2.waitKey(1)
        print("  [Thread] Display successful")
    except Exception as e:
        print(f"  [Thread] ERROR: {e}")

def test_child_thread_display():
    print("\n[Test 2] Displaying from child thread...")
    thread = threading.Thread(target=display_from_thread)
    thread.start()
    thread.join()
    time.sleep(2)
    cv2.destroyAllWindows()

# 测试3: 模拟GLib主循环环境（类似GStreamer）
def glib_thread_worker():
    print("  [GLib Thread] Starting GLib main loop...")
    loop = GLib.MainLoop()
    
    def display_from_glib():
        print("  [GLib Thread] Attempting to display...")
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "GLib Thread", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        try:
            cv2.imshow("Test - GLib Thread", frame)
            cv2.waitKey(1)
            print("  [GLib Thread] Display successful")
        except Exception as e:
            print(f"  [GLib Thread] ERROR: {e}")
        return False  # Don't repeat
    
    GLib.timeout_add(100, display_from_glib)
    GLib.timeout_add(3000, loop.quit)  # 3秒后退出
    loop.run()
    print("  [GLib Thread] GLib main loop stopped")

def test_glib_thread_display():
    print("\n[Test 3] Displaying from GLib thread (simulating GStreamer)...")
    thread = threading.Thread(target=glib_thread_worker, daemon=True)
    thread.start()
    thread.join(timeout=5)
    time.sleep(1)
    cv2.destroyAllWindows()

# 运行测试
if __name__ == '__main__':
    try:
        test_main_thread_display()
        test_child_thread_display()
        test_glib_thread_display()
        
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        print("Check console output above for errors")
        print("If you saw QBasicTimer/QObject errors, that's the problem!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        cv2.destroyAllWindows()

