# scrcpy 操控安卓虚拟机完整指南

## 概述
scrcpy 是一个开源的Android屏幕镜像和控制工具，可以将Android设备的屏幕显示到PC上，并支持键鼠操作。它特别适合用于自动化脚本开发、远程控制、屏幕录制等场景。

## 核心特性

### 主要优势
- **延迟极低**：支持60fps流畅显示
- **完全免费**：开源项目，无任何费用
- **跨平台支持**：Windows、macOS、Linux
- **键鼠控制**：直接用PC键鼠操作Android设备
- **屏幕录制**：支持录制操作过程
- **无需安装**：在Android设备上无需安装任何应用

### 技术原理
```text
Android设备 ←→ ADB连接 ←→ scrcpy ←→ PC显示和控制
```

## 安装配置

### macOS 环境安装

#### 方式1：Homebrew安装（推荐）
```bash
# 安装Homebrew（如果没有）
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 安装scrcpy
brew install scrcpy

# 安装ADB工具
brew install android-platform-tools
```

#### 方式2：手动下载
```bash
# 从GitHub下载最新版本
# https://github.com/Genymobile/scrcpy/releases
# 下载macOS版本并解压到PATH目录
```

### Windows 环境安装
```bash
# 使用Chocolatey
choco install scrcpy

# 或手动下载exe文件
# https://github.com/Genymobile/scrcpy/releases
```

### Linux 环境安装
```bash
# Ubuntu/Debian
sudo apt install scrcpy

# CentOS/RHEL
sudo yum install scrcpy

# Arch Linux
sudo pacman -S scrcpy
```

## 基础使用方法

### 1. USB连接控制
```bash
# 基本连接
scrcpy

# 指定设备（多设备时）
scrcpy --serial emulator-5554

# 设置窗口标题
scrcpy --window-title "我的手机"
```

### 2. WiFi连接控制
```bash
# 步骤1：USB连接并开启TCP/IP模式
adb tcpip 5555

# 步骤2：连接设备IP
adb connect 192.168.1.100:5555

# 步骤3：启动scrcpy
scrcpy
```

### 3. 常用参数配置
```bash
# 性能优化
scrcpy --max-fps 60 --bit-rate 8M

# 窗口大小控制
scrcpy --max-size 1920

# 录制屏幕
scrcpy --record screen.mp4

# 关闭音频
scrcpy --no-audio
```

## Python 多虚拟机并行控制

### 技术架构
```text
Python主程序
├── 虚拟机管理模块
│   ├── 虚拟机1 (scrcpy窗口1)
│   ├── 虚拟机2 (scrcpy窗口2)
│   └── 虚拟机N (scrcpy窗口N)
├── 任务调度模块
└── 自动化操作模块
```

### 核心实现代码

#### 1. 多虚拟机启动管理
```python
import subprocess
import time
import pyautogui
from typing import List

class ScrcpyManager:
    def __init__(self):
        self.vm_processes = []
        self.vm_windows = []
    
    def start_multiple_vms(self, device_serials: List[str]):
        """启动多个scrcpy实例"""
        for i, serial in enumerate(device_serials):
            # 为每个虚拟机启动独立的scrcpy窗口
            process = subprocess.Popen([
                'scrcpy',
                '--serial', serial,
                '--window-title', f'VM{i+1}',
                '--max-size', '800'  # 控制窗口大小
            ])
            self.vm_processes.append(process)
        
        # 等待所有窗口启动完成
        time.sleep(5)
        
        # 获取窗口引用
        self._get_vm_windows()
    
    def _get_vm_windows(self):
        """获取所有虚拟机窗口引用"""
        for i in range(len(self.vm_processes)):
            window_title = f'VM{i+1}'
            try:
                window = pyautogui.getWindowsWithTitle(window_title)[0]
                self.vm_windows.append(window)
            except IndexError:
                print(f"警告：未找到窗口 {window_title}")
    
    def close_all_vms(self):
        """关闭所有虚拟机"""
        for process in self.vm_processes:
            process.terminate()
        self.vm_processes.clear()
        self.vm_windows.clear()
```

#### 2. 并行任务执行
```python
import threading
from concurrent.futures import ThreadPoolExecutor

class ParallelTaskExecutor:
    def __init__(self, scrcpy_manager: ScrcpyManager):
        self.scrcpy_manager = scrcpy_manager
    
    def execute_parallel_tasks(self, tasks: List[dict]):
        """并行执行多个任务"""
        with ThreadPoolExecutor(max_workers=len(tasks)) as executor:
            # 为每个虚拟机分配任务
            futures = []
            for i, task in enumerate(tasks):
                if i < len(self.scrcpy_manager.vm_windows):
                    future = executor.submit(
                        self._execute_single_task,
                        i, task
                    )
                    futures.append(future)
            
            # 等待所有任务完成
            for future in futures:
                future.result()
    
    def _execute_single_task(self, vm_index: int, task: dict):
        """在指定虚拟机上执行单个任务"""
        window = self.scrcpy_manager.vm_windows[vm_index]
        
        # 激活窗口
        window.activate()
        time.sleep(0.5)
        
        # 执行具体任务
        if task['type'] == 'click':
            pyautogui.click(x=task['x'], y=task['y'])
        elif task['type'] == 'type':
            pyautogui.typewrite(task['text'])
        elif task['type'] == 'scroll':
            pyautogui.scroll(task['scroll_amount'])
        
        print(f"VM{vm_index+1} 完成任务: {task['description']}")
```

#### 3. 实际使用示例
```python
def main():
    # 初始化管理器
    manager = ScrcpyManager()
    
    # 定义虚拟机设备序列号
    device_serials = [
        'emulator-5554',  # 虚拟机1
        'emulator-5556',  # 虚拟机2
        'emulator-5558'   # 虚拟机3
    ]
    
    try:
        # 启动多个虚拟机
        print("正在启动虚拟机...")
        manager.start_multiple_vms(device_serials)
        print(f"成功启动 {len(manager.vm_windows)} 个虚拟机")
        
        # 定义并行任务
        tasks = [
            {'type': 'click', 'x': 100, 'y': 200, 'description': '点击登录按钮'},
            {'type': 'type', 'text': 'username', 'description': '输入用户名'},
            {'type': 'scroll', 'scroll_amount': -100, 'description': '向下滚动'}
        ]
        
        # 执行并行任务
        executor = ParallelTaskExecutor(manager)
        executor.execute_parallel_tasks(tasks)
        
        print("所有任务执行完成！")
        
    except Exception as e:
        print(f"执行过程中出现错误: {e}")
    
    finally:
        # 清理资源
        input("按回车键关闭所有虚拟机...")
        manager.close_all_vms()

if __name__ == "__main__":
    main()
```

## 高级功能配置

### 1. 性能优化参数
```bash
# 高帧率模式
scrcpy --max-fps 60 --bit-rate 12M

# 低延迟模式
scrcpy --max-fps 30 --bit-rate 4M

# 高质量模式
scrcpy --max-fps 60 --bit-rate 16M --max-size 1920
```

### 2. 窗口布局优化
```python
def optimize_window_layout(self):
    """优化多虚拟机窗口布局"""
    screen_width = pyautogui.size().width
    screen_height = pyautogui.size().height
    
    # 计算窗口大小和位置
    window_width = screen_width // 2
    window_height = screen_height // 2
    
    positions = [
        (0, 0),                    # 左上角
        (window_width, 0),         # 右上角
        (0, window_height),        # 左下角
        (window_width, window_height)  # 右下角
    ]
    
    # 重新排列窗口
    for i, window in enumerate(self.vm_windows):
        if i < len(positions):
            x, y = positions[i]
            window.moveTo(x, y)
            window.resizeTo(window_width, window_height)
```

### 3. 错误处理和重试机制
```python
def robust_task_execution(self, vm_index: int, task: dict, max_retries: int = 3):
    """带重试机制的任务执行"""
    for attempt in range(max_retries):
        try:
            self._execute_single_task(vm_index, task)
            return True
        except Exception as e:
            print(f"VM{vm_index+1} 任务执行失败 (尝试 {attempt+1}/{max_retries}): {e}")
            if attempt < max_retries - 1:
                time.sleep(2)  # 等待后重试
            else:
                print(f"VM{vm_index+1} 任务最终失败: {task['description']}")
                return False
```

## 实际应用场景

### 1. 批量测试场景
```python
# 同时测试多个应用版本
test_tasks = [
    {'vm': 0, 'action': 'install_app', 'app_path': 'app_v1.0.apk'},
    {'vm': 1, 'action': 'install_app', 'app_path': 'app_v1.1.apk'},
    {'vm': 2, 'action': 'install_app', 'app_path': 'app_v1.2.apk'}
]
```

### 2. 多账号操作场景
```python
# 同时操作多个账号
account_tasks = [
    {'vm': 0, 'username': 'user1', 'password': 'pass1'},
    {'vm': 1, 'username': 'user2', 'password': 'pass2'},
    {'vm': 2, 'username': 'user3', 'password': 'pass3'}
]
```

### 3. 性能对比测试
```python
# 同时运行性能测试
performance_tasks = [
    {'vm': 0, 'test': 'cpu_benchmark', 'config': 'low'},
    {'vm': 1, 'test': 'cpu_benchmark', 'config': 'medium'},
    {'vm': 2, 'test': 'cpu_benchmark', 'config': 'high'}
]
```

## 注意事项和最佳实践

### 1. 硬件要求
- **CPU**：建议多核处理器，每个虚拟机至少1个核心
- **内存**：每个虚拟机至少2GB RAM
- **存储**：SSD硬盘，提高虚拟机启动速度
- **网络**：稳定的网络连接，特别是WiFi模式

### 2. 性能优化建议
- 合理设置scrcpy参数，平衡质量和性能
- 避免同时运行过多虚拟机（建议不超过4个）
- 定期清理虚拟机缓存和临时文件
- 使用有线网络连接，减少延迟

### 3. 错误处理策略
- 实现完善的异常捕获和日志记录
- 设置任务超时机制，避免无限等待
- 实现自动重试和故障转移
- 监控虚拟机状态，及时处理异常

### 4. 安全注意事项
- 仅用于合法的开发和测试目的
- 不要在虚拟机中处理敏感信息
- 定期更新scrcpy和ADB工具
- 注意网络安全，避免暴露虚拟机

## 总结

scrcpy + Python 多虚拟机控制方案提供了强大的并行操作能力，特别适合需要批量处理、多账号操作、性能测试等场景。通过合理的架构设计和错误处理，可以实现稳定可靠的多虚拟机并行控制。

这种方案的优势在于：
- **真正的并行控制**：可以同时操作多个虚拟机
- **灵活的任务分配**：每个虚拟机执行不同的任务
- **高效的资源利用**：充分利用硬件资源
- **易于扩展**：可以轻松添加更多虚拟机

对于需要"同一时间使用Python对不同的虚拟机做不同的事情"的需求，这是一个非常理想的技术解决方案。
