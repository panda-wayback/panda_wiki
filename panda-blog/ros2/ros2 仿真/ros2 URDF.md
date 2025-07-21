# ROS2 仿真核心：URDF 机器人描述文件详解

> “如果我想在 Rviz 或 Gazebo 中看到我的机器人，第一步该做什么？”
>
> **答案是：为你的机器人创建一个 URDF 模型。**

URDF (Unified Robot Description Format) 是一个基于 XML 的文件，用于描述机器人模型的所有物理属性。它是 ROS 生态系统的基石，被广泛用于：

-   **可视化 (Rviz)**：实时显示机器人的 3D 模型和状态。
-   **仿真 (Gazebo)**：为机器人模型赋予物理属性（质量、惯性、碰撞），使其能在虚拟世界中与环境交互。
-   **运动学与动力学分析**：`robot_state_publisher` 等核心节点使用 URDF 来计算机器人各部件的位姿。

---

## 1. URDF 的两大核心：`link` 和 `joint`

一个机器人模型由两种核心组件构成：

-   **`<link>` (连杆)**：代表机器人的一个**物理部件**。它可以是任何形状，例如轮子、手臂、机身或传感器支架。每个 `link` 都有自己的坐标系。
-   **`<joint>` (关节)**：定义两个 `link` 之间的**连接关系**。它描述了子 `link` 相对于父 `link` 的运动方式（如旋转、滑动或固定）。

想象一下，你的手臂模型：
-   `上臂`、`前臂`、`手掌` 分别是三个独立的 `<link>`。
-   连接 `上臂` 和 `前臂` 的 `肘关节` 是一个 `<joint>`。
-   连接 `前臂` 和 `手掌` 的 `腕关节` 是另一个 `<joint>`。

---

## 2. `<link>`：机器人的“骨骼”

一个 `<link>` 主要由三个子标签定义，它们共同描述了这个部件的完整属性。

```xml
<link name="my_link">
    <!-- 1. 外观：在Rviz里长什么样 -->
    <visual>
        <geometry>
            <box size="0.1 0.2 0.3" />
        </geometry>
        <material name="blue">
            <color rgba="0.0 0.0 0.8 1.0" />
        </material>
    </visual>

    <!-- 2. 物理边界：用于碰撞检测 -->
    <collision>
        <geometry>
            <cylinder radius="0.05" length="0.4" />
        </geometry>
    </collision>

    <!-- 3. 动力学属性：用于物理仿真 -->
    <inertial>
        <mass value="1.0" />
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0"
                 izz="0.01" />
    </inertial>
</link>
```

-   **`<visual>` (视觉)**
    -   **作用**：定义了 link 在 Rviz 等可视化工具中的**外观**。
    -   **`geometry`**：可以是 `<box>`、`<cylinder>`、`<sphere>` 等基本形状，也可以是外部 3D 模型文件 `<mesh filename="path/to/your/model.dae" />`。
    -   **`material`**：定义颜色和纹理。

-   **`<collision>` (碰撞)**
    -   **作用**：定义了 link 在 Gazebo 等物理引擎中的**碰撞边界**。
    -   **关键点**：为了性能，`collision` 模型通常比 `visual` 模型**简单**。例如，一个复杂、高精度的机器人手臂 `visual` 模型，其 `collision` 模型可能只是一系列简单的圆柱体和球体。

-   **`<inertial>` (惯性)**
    -   **作用**：定义了 link 的**动力学参数**，如质量 (`mass`) 和惯性张量 (`inertia`)。
    -   **关键点**：这些值对于在 Gazebo 中进行**逼真的物理仿真**至关重要。如果缺失，物体在仿真中可能表现得像没有质量一样。

---

## 3. `<joint>`：连接“骨骼”的“关节”

`<joint>` 定义了两个 `link` 如何连接，以及它们如何相对运动。

```xml
<joint name="my_joint" type="revolute">
    <!-- 1. 连接的父子 Link -->
    <parent link="parent_link_name" />
    <child link="child_link_name" />

    <!-- 2. 子 Link 相对于父 Link 的静态位姿 -->
    <origin xyz="0.0 0.0 0.5" rpy="0 1.57 0" />

    <!-- 3. 运动轴 (对于非 fixed 关节) -->
    <axis xyz="0 0 1" />

    <!-- 4. 运动限制 (对于 revolute 和 prismatic 关节) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
</joint>
```

-   **`type` (关节类型)**：这是最重要的属性，决定了 link 的运动方式。
    -   `fixed`：**固定关节**。两个 link 完全固定，无相对运动。常用于连接传感器、摄像头等到机器人主体上。
    -   `revolute`：**旋转关节**。子 link 绕着 `axis` 定义的轴旋转，有角度限制（如机器人手臂的肘关节）。
    -   `continuous`：**连续旋转关节**。与 `revolute` 类似，但没有角度限制（如驱动轮）。
    -   `prismatic`：**滑动关节**。子 link 沿着 `axis` 轴线滑动，有位移限制（如升降装置）。
    -   `floating` / `planar`：更复杂的关节类型，不常用。

-   **`<parent>` 和 `<child>`**：定义了连接的两个 link。一个 link 可以是多个 joint 的 `parent`，但只能是一个 joint 的 `child`，这构成了机器人的**树状结构**。

-   **`<origin>`**：定义了**子 link 坐标系**相对于**父 link 坐标系**的静态**偏移量和旋转** (`xyz` 和 `rpy` Roll-Pitch-Yaw)。这是定义机器人结构的核心。

-   **`<axis>`**：定义了关节运动的轴。`xyz="0 0 1"` 表示绕 Z 轴旋转或滑动。

---

## 4. 实战：从零创建一个两轮小车 URDF

让我们创建一个简单的差速驱动小车。它包含：
-   一个长方体**机身** (`base_link`)。
-   两个**轮子** (`left_wheel_link`, `right_wheel_link`)，通过 `continuous` 关节连接到机身。
-   一个**支撑轮** (`caster_wheel_link`)，通过 `fixed` 关节连接到机身。

**完整 `two_wheeled_robot.urdf` 文件如下：**

```xml
<?xml version="1.0"?>
<robot name="two_wheeled_robot">

  <!-- ================== Links ================== -->

  <!-- Base Link (机身) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
        <mass value="5.0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Right Wheel Link (右轮) -->
  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Wheel Link (左轮) -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Caster Wheel Link (前支撑轮) -->
  <link name="caster_wheel_link">
      <visual>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
          <material name="gray"/>
      </visual>
      <collision>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
      </collision>
      <inertial>
          <mass value="0.2"/>
          <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
  </link>

  <!-- ================== Joints ================== -->

  <!-- Right Wheel Joint (右轮关节) -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0.0 -0.225 0.0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Left Wheel Joint (左轮关节) -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0.0 0.225 0.0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Caster Wheel Joint (支撑轮关节) -->
  <joint name="caster_wheel_joint" type="fixed">
      <parent link="base_link"/>
      <child link="caster_wheel_link"/>
      <origin xyz="0.25 0.0 -0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## 5. 在 ROS2 中可视化你的 URDF

光有 URDF 文件还不够，你还需要 ROS2 节点来解析它，并将其发布到系统中，这样 Rviz 才能看到。

1.  **`robot_state_publisher`**：
    -   读取 URDF 内容（从 `/robot_description` 参数）。
    -   订阅 `/joint_states` 话题，获取所有非 `fixed` 关节的当前角度或位置。
    -   根据 URDF 结构和关节状态，计算出所有 `link` 的 TF (Transform) 关系，并发布到 `/tf` 话题。

2.  **`joint_state_publisher_gui`**：
    -   提供一个图形界面，让你能手动拖动滑块来改变关节值。
    -   它会把这些值发布到 `/joint_states` 话题，非常适合在没有真实机器人或控制器时调试 URDF。

3.  **Rviz2**：
    -   添加 `RobotModel` 显示插件，它会订阅 `/robot_description` 来获取模型结构。
    -   添加 `TF` 显示插件，它会订阅 `/tf` 话题，并根据 TF 树来渲染所有 link 的正确位姿。

**一个简单的 Launch 文件 `display.launch.py` 来启动所有东西：**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # 找到你的 URDF 文件
    urdf_file_name = 'two_wheeled_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('your_package_name'), # <-- 修改为你的包名
        'urdf',
        urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. 启动 robot_state_publisher
        # 它会读取 /robot_description 参数中的 URDF 内容
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
        ),

        # 2. 启动 joint_state_publisher_gui
        # 提供一个 GUI 来发布 /joint_states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # 3. 启动 Rviz2
        # 加载预设的 Rviz 配置文件来显示模型
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', rviz_config_path] # 如果有配置文件，取消此行注释
        ),
    ])
```

---

## 6. 进阶：使用 XACRO 简化 URDF

当机器人模型变得复杂时，手写纯 URDF 会非常痛苦（大量重复、难以修改）。**XACRO (XML Macros)** 是 URDF 的超集，它提供了：
-   **常量和变量**：定义常用数值，如 `wheel_radius`。
-   **数学表达式**：在定义尺寸和位置时进行计算。
-   **宏 (macro)**：创建可重用的代码块，例如用一个宏来生成轮子，只需传入不同的参数即可。

**示例：使用 XACRO 定义轮子**

```xml
<!-- 定义常量 -->
<xacro:property name="wheel_radius" value="0.1" />
<xacro:property name="wheel_length" value="0.05" />

<!-- 定义一个宏 -->
<xacro:macro name="wheel_link" params="prefix">
    <link name="${prefix}_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
            </geometry>
            <material name="black"/>
        </visual>
        <!-- ... collision and inertial ... -->
    </link>
</xacro:macro>

<!-- 使用宏来创建两个轮子 -->
<xacro:wheel_link prefix="left"/>
<xacro:wheel_link prefix="right"/>
```

在实际项目中，**几乎总是使用 XACRO (`.xacro`) 文件来编写机器人模型**，然后在 launch 文件中将其转换为纯 URDF (`.urdf`) 再加载。

---

## 总结

-   URDF 是描述机器人物理模型的标准格式，核心是 `link` 和 `joint`。
-   `link` 描述了部件的**视觉、碰撞和惯性**属性。
-   `joint` 描述了 `link` 之间的**连接关系和运动方式**。
-   `robot_state_publisher` 和 `joint_state_publisher` 是在 Rviz 中可视化 URDF 的关键节点。
-   对于复杂模型，**强烈推荐使用 XACRO** 来提高可读性和可维护性。
