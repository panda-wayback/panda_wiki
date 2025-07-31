# URDF基础教程 - 第二课：认识collision

## URDF的层级结构

在开始学习collision之前，让我们回顾一下URDF的整体层级结构：

```
robot (机器人)
  ├── link (连杆)
  │   ├── visual (视觉)
  │   │   ├── geometry (几何形状)
  │   │   └── material (材质)
  │   ├── collision (碰撞)  ← 本节课重点
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

**本节课重点**：学习`collision`（碰撞）的配置和使用。

## 什么是collision？

**collision = 碰撞检测的形状**

`collision`定义了机器人在物理仿真中的碰撞边界，用于：
- **碰撞检测** - 防止机器人零件相互穿透
- **物理仿真** - 计算真实的物理交互
- **路径规划** - 避障算法使用碰撞信息

## collision vs visual 的区别

| 特性 | visual | collision |
|------|--------|-----------|
| **用途** | 显示外观 | 物理碰撞 |
| **形状** | 可以复杂 | 应该简单 |
| **性能** | 影响显示速度 | 影响仿真速度 |
| **精度** | 追求美观 | 追求效率 |

## 为什么collision要简单？

**性能考虑**：
- 碰撞检测需要实时计算
- 复杂形状计算量大，仿真变慢
- 简单形状（盒子、圆柱、球）计算快

**实际需求**：
- 碰撞检测不需要精确到每个细节
- 只要不穿透即可
- 简单形状足够满足需求

## collision的配置结构

```xml
<link name="零件名称">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- 几何形状 -->
    </geometry>
  </collision>
</link>
```

**说明**：
- `origin`：碰撞形状的位置和姿态
- `geometry`：碰撞形状的类型和尺寸

## 支持的几何形状

### 1. 盒子 (box)
```xml
<geometry>
  <box size="长 宽 高"/>
</geometry>
```

**适用场景**：
- 方形零件（车身、底座）
- 规则的长方体

### 2. 圆柱 (cylinder)
```xml
<geometry>
  <cylinder length="高度" radius="半径"/>
</geometry>
```

**适用场景**：
- 圆形零件（轮子、轴）
- 圆柱形传感器

### 3. 球体 (sphere)
```xml
<geometry>
  <sphere radius="半径"/>
</geometry>
```

**适用场景**：
- 球形零件（球轮、球关节）
- 简单的传感器外壳

### 4. 网格 (mesh)
```xml
<geometry>
  <mesh filename="package://包名/路径/文件.dae"/>
</geometry>
```

**注意**：虽然支持，但不推荐用于collision，因为：
- 计算复杂，性能差
- 通常用简单形状替代

## 实际应用示例

### 四轮小车的collision配置

```xml
<robot name="四轮小车">
  <!-- 车身 - 使用盒子 -->
  <link name="车身">
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- 轮子 - 使用圆柱 -->
  <link name="左前轮">
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <link name="右前轮">
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <link name="左后轮">
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <link name="右后轮">
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```

**说明**：
- 车身使用盒子，简单高效
- 轮子使用圆柱，旋转90度（1.5708弧度）
- 所有collision都配置了合适的origin

## collision的最佳实践

### 1. 形状选择原则
- **优先使用**：box、cylinder、sphere
- **避免使用**：复杂的mesh
- **考虑性能**：越简单越好

### 2. 尺寸设置
- **略大于visual**：确保完全覆盖
- **不要太夸张**：避免不必要的碰撞
- **考虑实际**：符合物理直觉

### 3. 位置调整
- **origin很重要**：确保碰撞形状位置正确
- **考虑旋转**：轮子需要旋转90度
- **测试验证**：在仿真中检查碰撞效果

## 常见错误

### 错误1：忘记配置collision
```xml
<!-- ❌ 错误：没有collision -->
<link name="轮子">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </visual>
  <!-- 缺少collision！ -->
</link>
```

**问题**：仿真中轮子没有碰撞检测，会穿透其他物体。

### 错误2：collision形状太复杂
```xml
<!-- ❌ 错误：使用复杂mesh -->
<collision>
  <geometry>
    <mesh filename="complex_wheel.dae"/>  <!-- 太复杂！ -->
  </geometry>
</collision>
```

**问题**：仿真速度慢，性能差。

### 错误3：collision尺寸太小
```xml
<!-- ❌ 错误：collision比visual小 -->
<visual>
  <geometry>
    <cylinder length="0.1" radius="0.05"/>  <!-- 实际大小 -->
  </geometry>
</visual>
<collision>
  <geometry>
    <cylinder length="0.08" radius="0.03"/>  <!-- 太小！ -->
  </geometry>
</collision>
```

**问题**：碰撞检测不准确，可能穿透。

## 总结

**collision的重要性**：
- **物理仿真必需** - 没有collision无法进行物理仿真
- **碰撞检测基础** - 所有碰撞检测都依赖collision
- **性能关键** - 影响仿真速度和准确性

**配置要点**：
- **形状简单** - 优先使用基本几何形状
- **尺寸合适** - 略大于visual，不要太夸张
- **位置正确** - 使用origin调整位置和姿态
- **测试验证** - 在仿真中检查效果

**下一课**：我们将学习inertial（惯性）配置，这是物理仿真的另一个重要部分！ 