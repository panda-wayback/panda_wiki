# ROS2 常用命令行工具备忘录

这份文档是一个快速参考指南，旨在帮助您快速查找并使用 ROS2 中最常用的命令行工具。

---

### 1. `ros2 topic list`

- **作用**: 列出当前 ROS2 网络中所有正在活动的话题名称。
- **语法**: `ros2 topic list [options]`

**示例**:
```bash
# 列出所有话题
ros2 topic list
```

**示例输出**:
```text
/chatter
/parameter_events
/rosout
```

---

### 2. `ros2 topic echo`

- **作用**: 订阅一个话题，并将其接收到的消息实时打印到控制台。这是调试时最常用的“监听”工具。
- **语法**: `ros2 topic echo <topic_name> [message_type]`

**示例**:
```bash
# 监听 /chatter 话题的消息
ros2 topic echo /chatter
```

**示例输出**:
```text
data: 'Hello, World! 1'
---
data: 'Hello, World! 2'
---
```

---

### 3. `ros2 topic pub`

- **作用**: 手动向一个话题发布单条消息。非常适合用来模拟发布者，以测试订阅者节点的行为。
- **语法**: `ros2 topic pub <topic_name> <message_type> '<yaml_data>' [options]`

**示例**:
```bash
# 向 /chatter 话题发布一条 String 类型的消息，内容为 "Hi there"
# --once 选项表示发布一次后即退出
ros2 topic pub --once /chatter std_msgs/msg/String '{data: "Hi there"}'
```

**示例输出**:
```text
Publisher new-publisher created, waiting for subscriber...
```
(此时，正在 `echo` 该话题的终端会显示出 `data: 'Hi there'`)

---

### 4. `ros2 service list`

- **作用**: 列出当前 ROS2 网络中所有可用的服务名称。
- **语法**: `ros2 service list [options]`

**示例**:
```bash
# 列出所有服务及其类型
ros2 service list -t
```

**示例输出**:
```text
/clear [std_srvs/srv/Empty]
/reset [std_srvs/srv/Empty]
/set_bool [std_srvs/srv/SetBool]
```

---

### 5. `ros2 service call`

- **作用**: 从命令行直接调用一个服务，并向其发送请求。这是手动测试服务功能的“遥控器”。
- **语法**: `ros2 service call <service_name> <service_type> '<yaml_data>'`

**示例**:
```bash
# 调用 /set_bool 服务，请求将布尔值设置为 true
ros2 service call /set_bool std_srvs/srv/SetBool '{data: true}'
```

**示例输出**:
```text
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
std_srvs.srv.SetBool_Response(success=True, message='Successfully set to true')
```

---

### 6. `ros2 interface show`

- **作用**: 显示一个具体的消息、服务或动作类型的数据结构。在你需要构造 `topic pub` 或 `service call` 的数据时，这个命令至关重要。
- **语法**: `ros2 interface show <type_name>`

**示例**:
```bash
# 查看 String 消息的结构
ros2 interface show std_msgs/msg/String
```

**示例输出**:
```text
# This is a simple string message.
string data
```

---

### 7. `ros2 topic info`

- **作用**: 显示一个话题的详细信息，包括它的发布者数量、订阅者数量以及各自的 QoS 配置。
- **语法**: `ros2 topic info <topic_name> [options]`

**示例**:
```bash
# 查看 /chatter 话题的详细信息，-v (verbose) 选项可以显示 QoS
ros2 topic info /chatter -v
```

**示例输出**:
```text
Type: std_msgs/msg/String

Publisher count: 1

Node name: talker
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: PUBLISHER
GID: 01.0f.3a...
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
  ...

Subscription count: 1

Node name: listener
Node namespace: /
Topic type: std_msgs/msg/String
Endpoint type: SUBSCRIPTION
GID: 01.0f.8b...
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
  ...
```
这个命令对于诊断节点间因 **QoS 不兼容**而导致的通信失败问题，非常非常有用。 