# ROS2 Launch 教程：自动化启动你的机器人系统

## 1. 为什么需要 Launch 文件？告别手动启动

想象一下，你的机器人项目包含 5 个核心节点：相机驱动、激光雷达驱动、里程计估算、路径规划和控制节点。

**没有 `launch` 文件，你需要**：
1.  打开 5 个独立的终端。
2.  在每个终端中，手动输入 `ros2 run <package_name> <node_name>` 命令。
3.  如果需要为某个节点传递参数，命令会变得更长、更复杂，而且容易出错。
4.  每次重启系统，都需要重复以上所有步骤。

**`launch` 文件就是为了解决这个问题的自动化工具。** 它是一个 Python 脚本，允许你用编程的方式来**描述、配置和启动**一个或多个 ROS2 节点，实现“一键启动”整个系统。

**核心优势**：
- **自动化 (Automation)**：一键启动、停止和监控整个节点系统。
- **可配置化 (Configuration)**：轻松为节点设置参数、命名空间和话题重映射。
- **可复用性 (Reusability)**：可以将复杂的启动逻辑封装成一个可重用的模块。

---

## 2. Launch 文件的核心概念

ROS2 的 `launch` 文件就是一个返回 `LaunchDescription` 对象的 Python 函数。

- **`LaunchDescription`**: 启动描述对象。你可以把它看作一个“任务清单”，里面包含了所有需要执行的“动作”。
- **`Action`**: 动作。这是任务清单中的一个具体条目，例如“启动一个节点”、“设置一个参数”或“包含另一个launch文件”。

最常用的 `Action` 就是 `launch_ros.actions.Node`。

---

## 3. 基础用法：启动你的第一个节点

让我们来看一个最简单的 `launch` 文件，它只启动一个节点。

**文件路径**: `my_package/launch/my_first_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 创建一个 LaunchDescription 对象
    ld = LaunchDescription()
    
    # 1. 定义要启动的节点
    talker_node = Node(
        package='demo_nodes_py',      # 节点所在的包
        executable='talker',          # 节点的可执行文件
        name='my_talker'              # 启动后节点的名称 (可选)
    )
    
    # 2. 将节点 Action 添加到 LaunchDescription
    ld.add_action(talker_node)
    
    return ld
```
*这个 `launch` 文件等同于手动在终端中运行 `ros2 run demo_nodes_py talker --ros-args -r __node:=my_talker`。*

---

## 4. 进阶用法：管理复杂系统

### ① 同时启动多个节点

`LaunchDescription` 可以像一个列表一样，包含任意多个 `Action`。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 可以直接在构造函数中传入一个 Action 列表
    return LaunchDescription([
        # 启动 talker 节点
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='my_talker'
        ),
        # 同时启动 listener 节点
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='my_listener'
        )
    ])
```

### ② 传递参数 (Parameters)

这是 `launch` 文件最有用的功能之一。你可以在启动时为节点动态配置参数。

```python
# ...
        Node(
            package='my_package',
            executable='my_node',
            name='my_configurable_node',
            # 'parameters' 字段接收一个字典列表
            parameters=[
                {'param_name_1': 'param_value_1'},
                {'param_name_2': 123},
                {'some_bool_param': True}
            ]
        )
# ...
```

### ③ 设置命名空间 (Namespaces)

为节点设置命名空间，可以有效避免不同模块间的命名冲突。

```python
# ...
        Node(
            package='my_package',
            executable='my_node',
            # 使用 namespace 字段
            namespace='robot1',
            name='my_namespaced_node'
        )
# ...
```
*这个节点启动后的完全限定名将是 `/robot1/my_namespaced_node`。*

### ④ 话题重映射 (Remapping)

重映射允许你改变节点订阅或发布的话题名称，实现节点间的“解耦”。

```python
# ...
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='my_listener',
            # 'remappings' 字段接收一个元组列表
            remappings=[
                # 将节点内部的 'chatter' 话题，重映射到外部的 '/my_chatter_topic'
                ('chatter', '/my_chatter_topic')
            ]
        )
# ...
```
*现在，这个 `listener` 节点会去监听 `/my_chatter_topic`，而不是默认的 `/chatter`。*

---

## 5. 高级技巧

### ① 使用启动参数 (Launch Arguments)

让你的 `launch` 文件更具灵活性，可以在**运行时**从外部传入值。

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 声明一个启动参数 'robot_name'，并设置默认值
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot'
    )

    # 2. 使用 LaunchConfiguration 来获取参数的值
    robot_name = LaunchConfiguration('robot_name')

    # 3. 在 Node 的定义中使用这个值
    my_node = Node(
        package='my_package',
        executable='my_node',
        # 将节点的命名空间设置为传入的 robot_name
        namespace=robot_name
    )

    return LaunchDescription([
        robot_name_arg,
        my_node
    ])
```

**如何使用？**
```bash
# 不提供参数，将使用默认值 'default_robot'
ros2 launch my_package my_launch.py

# 提供参数，覆盖默认值
ros2 launch my_package my_launch.py robot_name:=my_awesome_robot
```

### ② 包含其他 Launch 文件

`IncludeLaunchDescription` Action 允许你像导入 Python 模块一样，在一个 `launch` 文件中包含另一个，实现配置的模块化。

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取另一个包的 launch 文件路径
    other_package_path = get_package_share_directory('other_package')
    other_launch_file = os.path.join(other_package_path, 'launch', 'other_launch.py')

    # 定义包含动作
    include_other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file)
    )

    return LaunchDescription([
        include_other_launch
        # ... 你自己的其他节点
    ])
```

---

## 6. 如何运行 Launch 文件

使用 `ros2 launch` 命令来执行你的 `launch` 文件。

**语法**: `ros2 launch <package_name> <launch_file_name> [launch_arguments]`

**示例**:
```bash
# 运行 my_package 包中的 my_awesome_launch.py
ros2 launch my_package my_awesome_launch.py

# 运行并传递启动参数
ros2 launch my_package my_launch_with_args.py some_arg:=some_value
```

---

## 7. 总结与模板

下面是一个包含了上述大部分功能的 `launch` 文件模板，你可以将其作为你未来项目的起点。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取常用包的路径
    my_package_path = get_package_share_directory('my_package')

    # --- 1. 声明启动参数 ---
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='True',
        description='Whether to use simulation time'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='The name of the robot'
    )

    # --- 2. 获取启动参数的值 ---
    use_simulation = LaunchConfiguration('use_simulation')
    robot_name = LaunchConfiguration('robot_name')

    # --- 3. 定义节点 ---
    my_driver_node = Node(
        package='my_package',
        executable='driver_node',
        name='my_driver',
        namespace=robot_name,
        parameters=[{'use_sim_time': use_simulation}],
        remappings=[
            ('input_topic', 'remapped_input')
        ]
    )

    my_logic_node = Node(
        package='my_package',
        executable='logic_node',
        name='my_logic',
        namespace=robot_name,
        parameters=[{'use_sim_time': use_simulation}]
    )

    # --- 4. 包含其他 Launch 文件 ---
    # other_launch = IncludeLaunchDescription(...)
    
    # --- 5. 创建 LaunchDescription ---
    return LaunchDescription([
        # 添加启动参数声明
        use_simulation_arg,
        robot_name_arg,

        # 添加要启动的节点
        my_driver_node,
        my_logic_node,
        
        # 添加其他包含的 launch 文件
        # other_launch
    ])
``` 