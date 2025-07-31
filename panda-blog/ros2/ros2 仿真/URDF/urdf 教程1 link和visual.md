# URDF基础教程 - 第一课：认识link和visual

## URDF的层级结构

在开始学习之前，让我们先了解URDF的整体层级结构：

```text
robot (机器人)
  ├── link (连杆)
  │   ├── visual (视觉)  ← 本节课重点
  │   │   ├── geometry (几何形状)
  │   │   └── material (材质)
  │   ├── collision (碰撞)
  │   │   └── geometry (几何形状)
  │   └── inertial (惯性)
  │       ├── mass (质量)
  │       ├── inertia (转动惯量)
  │       └── origin (原点)
  └── joint (关节)
      ├── parent (父连杆)
      ├── child (子连杆)
      ├── origin (原点)
      └── axis (轴)
```

**本节课重点**：学习`link`（连杆）和`visual`（视觉）的配置和使用。

## 什么是link？

**link = 一个零件**

`link`是URDF中最基本的单位，代表一个独立的零件，比如：
- 一个盒子
- 一个圆柱
- 一个球体

## 什么是visual？

**visual = 零件的外观**

`visual`定义了零件在RViz中的显示效果，包括：
- **几何形状** - 零件的外形
- **材质** - 零件的颜色

## link的基本结构

```xml
<link name="零件名称">
  <visual>
    <geometry>
      <!-- 几何形状 -->
    </geometry>
    <material name="材质名称">
      <!-- 材质属性 -->
    </material>
  </visual>
</link>
```

**说明**：
- `name`：零件的唯一标识符
- `geometry`：零件的几何形状
- `material`：零件的材质属性

## 支持的几何形状

### 1. 盒子 (box)
```xml
<geometry>
  <box size="长 宽 高"/>
</geometry>
```

**适用场景**：
- 方形零件
- 规则的长方体

**示例**：`<box size="0.5 0.3 0.2"/>` 表示长0.5米、宽0.3米、高0.2米的长方体

### 2. 圆柱 (cylinder)
```xml
<geometry>
  <cylinder length="高度" radius="半径"/>
</geometry>
```

**适用场景**：
- 圆形零件
- 旋转对称的结构

**示例**：`<cylinder length="0.1" radius="0.05"/>` 表示高0.1米、半径0.05米的圆柱

### 3. 球体 (sphere)
```xml
<geometry>
  <sphere radius="半径"/>
</geometry>
```

**适用场景**：
- 球形零件
- 完全对称的结构

**示例**：`<sphere radius="0.05"/>` 表示半径0.05米的球体



## 材质配置

### 基本材质
```xml
<material name="红色">
  <color rgba="1 0 0 1"/>
</material>
```

**参数说明**：
- `rgba`：红、绿、蓝、透明度（0-1）
- `name`：材质名称，可以在多个link中重复使用

### 常用颜色
```xml
<!-- 红色 -->
<material name="红色">
  <color rgba="1 0 0 1"/>
</material>

<!-- 绿色 -->
<material name="绿色">
  <color rgba="0 1 0 1"/>
</material>

<!-- 蓝色 -->
<material name="蓝色">
  <color rgba="0 0 1 1"/>
</material>

<!-- 黑色 -->
<material name="黑色">
  <color rgba="0 0 0 1"/>
</material>

<!-- 白色 -->
<material name="白色">
  <color rgba="1 1 1 1"/>
</material>
```

## 实际应用示例

### 一个简单的零件

```xml
<?xml version="1.0"?>
<robot name="简单零件">
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
</robot>
```

**说明**：
- 只有一个车身零件
- 使用蓝色盒子形状
- 在RViz中会显示一个蓝色的长方体



## 在RViz中查看效果

### 1. 保存URDF文件
将上面的代码保存为`robot.urdf`

### 2. 启动RViz
```bash
ros2 launch urdf_tutorial display.launch.py model:=path/to/robot.urdf
```

### 3. 查看效果
- 在RViz中应该能看到一个蓝色的长方体

**注意**：如果看不到模型，请检查URDF文件格式是否正确

## 常见错误

### 错误1：忘记配置visual
```xml
<!-- ❌ 错误：link没有visual -->
<link name="车身">
  <!-- 缺少visual！ -->
</link>
```

**问题**：在RViz中看不到任何东西。

### 错误2：几何形状参数错误
```xml
<!-- ❌ 错误：box参数格式错误 -->
<geometry>
  <box size="0.5, 0.3, 0.2"/>  <!-- 不能用逗号！ -->
</geometry>
```

**问题**：URDF解析失败。

### 错误3：材质名称重复
```xml
<!-- ❌ 错误：材质名称重复 -->
<material name="红色">
  <color rgba="1 0 0 1"/>
</material>
<material name="红色">  <!-- 重复了！ -->
  <color rgba="0 1 0 1"/>
</material>
```

**问题**：后面的材质会覆盖前面的，导致颜色显示错误。

## 最佳实践

### 1. 命名规范
- **link名称**：使用有意义的名称（如"车身"而不是"link1"）
- **材质名称**：使用颜色名称（如"红色"、"蓝色"）
- **避免特殊字符**：不要使用空格、中文等

### 2. 尺寸设置
- **使用米为单位**：URDF使用国际单位制
- **合理的尺寸**：不要太大或太小
- **保持比例**：各个零件的比例要合理

### 3. 颜色选择
- **对比明显**：选择容易看到的颜色
- **避免透明**：alpha值设为1，避免显示问题

## 总结

**link和visual的作用**：
- **基础单位** - link是URDF的基本组成单位
- **显示外观** - visual决定了零件在RViz中的显示效果

**配置要点**：
- **几何形状** - 选择合适的形状（box/cylinder/sphere）
- **材质颜色** - 使用rgba设置颜色
- **命名规范** - 使用有意义的名称

**下一步**：在下一课中，我们将学习joint（关节），将多个link连接起来！ 