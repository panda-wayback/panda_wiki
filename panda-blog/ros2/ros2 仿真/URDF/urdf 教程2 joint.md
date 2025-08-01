# URDF基础教程 - 第二课：认识joint

## URDF的层级结构

在开始学习之前，让我们回顾一下URDF的整体层级结构：

```text
robot (机器人)
  ├── link (连杆)
  │   ├── visual (视觉)
  │   │   ├── geometry (几何形状)
  │   │   └── material (材质)
  │   ├── collision (碰撞)
  │   │   └── geometry (几何形状)
  │   └── inertial (惯性)
  │       ├── mass (质量)
  │       ├── inertia (转动惯量)
  │       └── origin (原点)
  └── joint (关节)  ← 本节课重点
      ├── parent (父连杆)
      ├── child (子连杆)
      ├── origin (原点)
      └── axis (轴)
```

**本节课重点**：学习`joint`（关节）的配置和使用。

## 什么是joint？

**joint = 连接两个零件的关节**

`joint`定义了如何将两个link连接起来，包括：
- **连接关系** - 哪个零件连接哪个零件
- **关节类型** - 如何运动（固定、旋转、移动等）
- **运动限制** - 运动的范围和速度

## joint的基本结构

```xml
<joint name="关节名称" type="关节类型">
  <parent link="父零件名称"/>
  <child link="子零件名称"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

**说明**：
- `name`：关节的唯一标识符
- `type`：关节的类型（fixed、revolute、prismatic等）
- `parent`：父零件（基础零件）
- `child`：子零件（被连接的零件）
- `origin`：关节的位置和姿态
- `axis`：关节的运动轴

## 关节类型

### 1. fixed（固定关节）
```xml
<joint name="固定关节" type="fixed">
  <parent link="基础"/>
  <child link="零件"/>
</joint>
```

**特点**：
- 两个零件完全固定在一起
- 不能相对运动
- 常用于连接多个零件

### 2. revolute（旋转关节）
```xml
<joint name="旋转关节" type="revolute">
  <parent link="基础"/>
  <child link="零件"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
</joint>
```

**特点**：
- 绕轴旋转
- 有运动范围限制
- 常用于轮子、机械臂关节

### 3. prismatic（移动关节）
```xml
<joint name="移动关节" type="prismatic">
  <parent link="基础"/>
  <child link="零件"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="100" velocity="0.5"/>
</joint>
```

**特点**：
- 沿轴移动
- 有运动范围限制
- 常用于直线运动机构

## parent和child的关系

### 理解连接关系
```text
parent link (父零件) ← joint (关节) → child link (子零件)
```

**说明**：
- **parent**：基础零件，通常不动
- **child**：被连接的零件，可以相对parent运动
- **joint**：定义如何连接和运动

### 实际例子
```xml
<!-- 车身是基础，轮子连接在车身上 -->
<joint name="左前轮关节" type="revolute">
  <parent link="车身"/>    <!-- 车身是基础 -->
  <child link="左前轮"/>   <!-- 轮子连接在车身上 -->
</joint>
```

## origin参数

### 位置设置
```xml
<origin xyz="x y z"/>
```

**说明**：
- `x`：前后位置（前为正）
- `y`：左右位置（左为正）
- `z`：上下位置（上为正）

### 姿态设置
```xml
<origin rpy="roll pitch yaw"/>
```

**说明**：
- `roll`：绕x轴旋转（弧度）
- `pitch`：绕y轴旋转（弧度）
- `yaw`：绕z轴旋转（弧度）

## axis参数

### 运动轴设置
```xml
<axis xyz="x y z"/>
```

**常用设置**：
- `xyz="0 0 1"`：绕z轴旋转（垂直旋转）
- `xyz="0 1 0"`：绕y轴旋转（左右旋转）
- `xyz="1 0 0"`：绕x轴旋转（前后旋转）

## 实际应用示例

### 四轮小车的joint配置

```xml
<?xml version="1.0"?>
<robot name="四轮小车">
  <!-- 车身 -->
  <link name="车身">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="蓝色">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  
  <!-- 左前轮 -->
  <link name="左前轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- 右前轮 -->
  <link name="右前轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- 左后轮 -->
  <link name="左后轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- 右后轮 -->
  <link name="右后轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- 连接关节 -->
  <joint name="左前轮关节" type="revolute">
    <parent link="车身"/>
    <child link="左前轮"/>
    <origin xyz="0.2 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  
  <joint name="右前轮关节" type="revolute">
    <parent link="车身"/>
    <child link="右前轮"/>
    <origin xyz="0.2 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  
  <joint name="左后轮关节" type="revolute">
    <parent link="车身"/>
    <child link="左后轮"/>
    <origin xyz="-0.2 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  
  <joint name="右后轮关节" type="revolute">
    <parent link="车身"/>
    <child link="右后轮"/>
    <origin xyz="-0.2 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
</robot>
```

**说明**：
- 车身作为基础（parent）
- 四个轮子都连接到车身上（child）
- 使用revolute关节，轮子可以旋转
- 轮子位置在车身四个角

## 在RViz中查看效果

### 1. 保存URDF文件
将上面的代码保存为`robot.urdf`

### 2. 启动RViz
```bash
ros2 launch urdf_tutorial display.launch.py model:=path/to/robot.urdf
```

### 3. 查看效果
- 在RViz中应该能看到完整的四轮小车
- 轮子连接在车身上，形成完整结构
- 可以使用Joint State Publisher控制轮子旋转

## 常见错误

### 错误1：忘记配置joint
```xml
<!-- ❌ 错误：link没有连接 -->
<link name="车身">...</link>
<link name="轮子">...</link>
<!-- 缺少joint！ -->
```

**问题**：在RViz中只能看到两个独立的零件，没有连接关系。

### 错误2：parent/child关系错误
```xml
<!-- ❌ 错误：parent和child写反了 -->
<joint name="关节" type="revolute">
  <parent link="轮子"/>  <!-- 应该是车身！ -->
  <child link="车身"/>   <!-- 应该是轮子！ -->
</joint>
```

**问题**：连接关系错误，可能导致显示异常。

### 错误3：关节类型选择错误
```xml
<!-- ❌ 错误：轮子用fixed关节 -->
<joint name="轮子关节" type="fixed">  <!-- 应该是revolute！ -->
  <parent link="车身"/>
  <child link="轮子"/>
</joint>
```

**问题**：轮子无法旋转，失去了运动功能。

## 最佳实践

### 1. 关节命名
- **使用描述性名称**：如"左前轮关节"而不是"joint1"
- **包含位置信息**：如"左前"、"右后"等
- **避免特殊字符**：不要使用空格、中文等

### 2. 连接关系
- **车身作为基础**：通常车身是parent
- **运动零件作为child**：轮子、机械臂等是child
- **层次清晰**：避免复杂的连接关系

### 3. 关节类型选择
- **fixed**：固定连接，不运动
- **revolute**：旋转运动（轮子、关节）
- **prismatic**：直线运动（滑块、升降）

## 总结

**joint的重要性**：
- **连接基础** - 没有joint就没有完整的机器人结构
- **运动定义** - joint决定了零件的运动方式
- **层次关系** - 通过parent/child建立零件间的层次

**配置要点**：
- **关节类型** - 选择合适的类型（fixed/revolute/prismatic）
- **连接关系** - 正确设置parent和child
- **位置设置** - 使用origin调整关节位置
- **运动轴** - 使用axis设置运动方向

**下一步**：在下一课中，我们将学习collision（碰撞），让机器人具有物理碰撞检测功能！ 