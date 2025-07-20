# ROS2 核心命令行工具教程: topic echo & service call

## 1. 为什么需要命令行工具？开发者的“听诊器”

如果说 ROS2 节点是机器人的“器官”，那么命令行工具就是开发者的“听诊器”和“手术刀”。没有它们，你将无法：
- **诊断问题**：节点发布的数据是否正确？服务是否按预期响应？
- **监控状态**：实时查看机器人的速度、位置和传感器数据。
- **手动测试**：在不编写完整客户端节点的情况下，手动触发一个服务或发布一条消息。

本教程旨在演示 ROS2 中两个最核心、最常用的命令行调试工具：
- `ros2 topic echo`: **监听**节点间的实时数据流。
- `ros2 service call`: **触发**一个节点执行特定任务。

---

## 2. `ros2 topic echo`: 节点世界的“窃听器”

`topic echo` 可能是你最常用的 ROS2 命令。它订阅一个指定的话题，并将接收到的所有消息实时打印到控制台。

#### 核心用途

- **验证发布者**：确认你的节点是否正在发布数据。
- **检查数据内容**：查看消息的实时值是否符合预期。
- **理解系统**：通过“窃听”核心话题，快速了解系统的运行状态。

#### 如何使用？

**基础用法**：
```bash
# 语法: ros2 topic echo <topic_name>
# 示例: 监听一个名为 /chatter 的简单字符串话题
ros2 topic echo /chatter
```

**输出示例**：
```text
data: 'Hello, World! 1'
---
data: 'Hello, World! 2'
---
data: 'Hello, World! 3'
---
```

**常用选项**：
- **手动指定消息类型**: 如果 ROS2 无法自动推断话题类型，你需要手动指定。
  ```bash
  # 语法: ros2 topic echo <topic_name> <message_type>
  ros2 topic echo /chatter std_msgs/msg/String
  ```
- **只打印一次**: 使用 `--once` 选项，命令会在接收到第一条消息后自动退出。
  ```bash
  ros2 topic echo /chatter --once
  ```

---

## 3. `ros2 service call`: 远程“遥控器”

`service call` 允许你从命令行直接调用一个服务，并向其发送请求数据。它就像一个万能遥控器，可以让你手动触发节点的功能。

#### 核心用途

- **测试服务功能**：在不编写客户端代码的情况下，验证你的服务端逻辑。
- **手动控制系统**：触发一次性的任务，例如“开启/关闭某个功能”。
- **改变节点状态**：调用服务来改变节点的内部参数或状态。

#### 如何使用？

**分解步骤**：

1.  **找到服务**：首先，你需要知道服务的名称和类型。
    ```bash
    # 列出所有可用的服务及其类型
    ros2 service list -t
    ```
    输出可能像这样：
    ```text
    /set_bool [std_srvs/srv/SetBool]
    ...
    ```

2.  **查看请求结构**：你需要知道如何构造请求数据。
    ```bash
    # 语法: ros2 interface show <service_type>
    # 查看“设置布尔值”服务的结构
    ros2 interface show std_srvs/srv/SetBool
    ```
    输出会告诉你请求（Request）和响应（Response）的字段：
    ```text
    # Request
    bool data   # The value to set
    ---
    # Response
    bool success  # Indicates if the service call was successful
    string message # A human-readable success message
    ```

3.  **构造并发送请求**：现在，你可以用 YAML 格式构造请求数据并发送。
    ```bash
    # 语法: ros2 service call <service_name> <service_type> '<request_data_in_yaml>'
    # 示例: 请求将布尔值设置为 true
    ros2 service call /set_bool std_srvs/srv/SetBool '{data: true}'
    ```
    **注意**：YAML 数据必须用单引号 `'` 包裹。

**输出示例**：
```text
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
std_srvs.srv.SetBool_Response(success=True, message='Successfully set to true')
```

---

## 4. 综合调试场景：一个真实的例子

**场景**：你写了一个节点来控制一个灯（通过 `/light_switch` 服务），但灯没亮。你需要找出问题所在。

**调试流程**：

1.  **第一步：检查灯的状态话题**
    *   **工具**：`ros2 topic echo`
    *   **命令**：`ros2 topic echo /light_status`
    *   **分析**：查看灯的当前状态（例如，输出 `data: 'off'`）。这确认了状态话题是正常的。

2.  **第二步：手动尝试开灯**
    *   **工具**：`ros2 service call`
    *   **命令**：`ros2 service call /light_switch std_srvs/srv/SetBool '{data: true}'`
    *   **分析**：
        *   **如果服务响应 `success: True` 并且灯亮了**：说明控制灯的服务本身是好的。问题可能出在你用来控制灯的那个客户端节点上。
        *   **如果服务调用失败或超时**：说明问题出在提供 `/light_switch` 服务的那个节点本身，它可能崩溃了或逻辑有误。

3.  **第三步：再次检查状态**
    *   **工具**：`ros2 topic echo --once /light_status`
    *   **命令**：再次监听状态话题。
    *   **分析**：如果此时状态变为 `data: 'on'`，则进一步证明了服务是有效的。

通过这几步，你可以将一个模糊的“灯不亮”的问题，精确定位到是“客户端控制节点的问题”还是“服务端本身的问题”。

---

## 5. 总结与备忘录

| 命令 | 作用 | 核心用途 |
| :--- | :--- | :--- |
| `ros2 topic list` | 列出所有活动话题 | 发现系统中有哪些数据流 |
| `ros2 topic echo <topic>`| **监听**话题数据 | 检查数据内容、确认发布者状态 |
| `ros2 topic pub <topic>` | **发布**单条消息 | 手动模拟发布者，测试订阅者 |
| `ros2 service list` | 列出所有活动服务 | 发现系统中有哪些可用功能 |
| `ros2 service call <svc>`| **触发**服务调用 | 手动测试服务端功能 |
| `ros2 interface show <type>`| 查看消息/服务结构 | 了解如何构造请求数据 |
| `ros2 topic info -v <topic>`| 查看话题详细信息 | **调试 QoS 兼容性问题** |

熟练掌握这些命令行工具，是提升 ROS2 开发和调试效率的关键。 