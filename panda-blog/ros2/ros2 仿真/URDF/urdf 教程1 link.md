# URDF基础教程 - 第一课：认识link

## URDF的层级结构

这是URDF最核心的概念 - 所有内容都按照这个层级结构组织：

```
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
  └── joint (关节)
      ├── parent (父连杆)
      ├── child (子连杆)
      ├── origin (原点)
      └── axis (轴)
```

**重要**：每个层级都必须按照这个顺序嵌套，不能跳过或颠倒。

**说明**：
- **visual**：零件的外观（用于显示）
- **collision**：零件的碰撞形状（用于物理仿真）
- **inertial**：零件的物理属性（质量、惯性）
- **joint**：连接零件的关节（用于运动）

## 什么是link？

**link = 机器人的零件/组件**

每个`link`相当于机器人的一个零件：

- **齿轮** - 一个link
- **轮子** - 一个link  
- **手臂** - 一个link
- **摄像头** - 一个link
- **传感器** - 一个link

## link的特点

**作用**：定义机器人的身体部分，每个link都是独立的零件。

**重要特性**：相同的零件可以重复使用！

## 基础URDF的限制

您想的这种写法很直观，但是**URDF不支持直接引用visual**：

```xml
<!-- ❌ 这样是不行的 -->
<visual name="轮子">
  <geometry>
    <cylinder length="0.1" radius="0.05"/>
  </geometry>
  <material name="黑色"/>
</visual>

<link name="左前轮">
  <visual name="轮子"/>  <!-- ❌ URDF不支持这种引用 -->
</link>
```

**URDF的限制**：
- visual不能单独定义和引用
- 每个link必须完整定义自己的visual
- 只有material可以重复使用

**这确实不够方便！**

想象一下，如果要修改轮子的形状：
- 需要修改4个地方（左前轮、右前轮、左后轮、右后轮）
- 容易出错，可能忘记修改某个轮子
- 代码冗长，维护困难

这就是为什么ROS社区开发了Xacro宏的原因！

## 正确的做法

```xml
<robot name="四轮小车">
  <!-- 定义可重用的材质 -->
  <material name="黑色">
    <color rgba="0 0 0 1"/>
  </material>
  
  <material name="红色">
    <color rgba="1 0 0 1"/>
  </material>
  
  <!-- 车身 -->
  <link name="车身">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="红色"/>
    </visual>
  </link>
  
  <!-- 四个轮子（必须重复写visual） -->
  <link name="左前轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色"/>
    </visual>
  </link>
  
  <link name="右前轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色"/>
    </visual>
  </link>
  
  <link name="左后轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色"/>
    </visual>
  </link>
  
  <link name="右后轮">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="黑色"/>
    </visual>
  </link>
</robot>
```

**说明**：
- URDF要求每个link完整定义自己的visual
- 只有material可以重复使用
- 要实现真正的复用，需要使用Xacro宏



## 总结

**link的好处**：
- **模块化** - 每个零件独立定义
- **可重用** - 相同的零件可以复制使用
- **易维护** - 修改一个零件，其他相同零件不受影响

**link的限制**：
- **代码重复** - 相同的零件必须重复写代码
- **维护困难** - 修改形状需要改多个地方
- **容易出错** - 可能忘记修改某个零件

**解决方案**：在后续课程中学习Xacro宏可以实现真正的代码复用！

下一课我们将学习visual、geometry和material的详细用法！
