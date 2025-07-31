# URDF层级结构详解

## 整体结构（从外到内）

```
robot (机器人)
  └── link (连杆)
      └── visual (视觉)
          ├── geometry (几何形状)
          └── material (材质)
```

## 第一层：robot（机器人）

这是最外层的容器，所有内容都放在这里面。

```xml
<?xml version="1.0"?>
<robot name="我的机器人">
    <!-- 这里放所有内容 -->
</robot>
```

**作用**：定义整个机器人，给它起个名字。

## 第二层：link（连杆）

机器人由多个连杆组成，每个连杆是机器人的一个身体部分。

```xml
<robot name="我的机器人">
    <link name="底座">
        <!-- 连杆的内容 -->
    </link>
    
    <link name="手臂">
        <!-- 连杆的内容 -->
    </link>
</robot>
```

**作用**：定义机器人的身体部分，比如底座、手臂、轮子等。

## 第三层：visual（视觉）

每个连杆需要定义它的外观，这就是视觉部分。

```xml
<link name="底座">
    <visual>
        <!-- 视觉内容 -->
    </visual>
</link>
```

**作用**：定义这个连杆在屏幕上显示的样子。

## 第四层：geometry（几何形状）

视觉部分需要定义具体的形状。

```xml
<visual>
    <geometry>
        <box size="1 1 1"/>
    </geometry>
</visual>
```

**作用**：定义具体的形状，比如立方体、圆柱体、球体。

## 第五层：material（材质）

给形状添加颜色。

```xml
<visual>
    <geometry>
        <box size="1 1 1"/>
    </geometry>
    <material name="红色">
        <color rgba="1 0 0 1"/>
    </material>
</visual>
```

**作用**：定义颜色和外观。

## 完整示例

让我们看一个完整的例子：

```xml
<?xml version="1.0"?>
<robot name="简单机器人">
    
    <!-- 定义材质 -->
    <material name="红色">
        <color rgba="1 0 0 1"/>
    </material>
    
    <!-- 第一个连杆 -->
    <link name="底座">
        <visual>
            <geometry>
                <box size="1 1 0.5"/>
            </geometry>
            <material name="红色"/>
        </visual>
    </link>
    
    <!-- 第二个连杆 -->
    <link name="手臂">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.8"/>
            </geometry>
            <material name="红色"/>
        </visual>
    </link>
    
</robot>
```

## 层级关系总结

1. **robot**：最外层，包含整个机器人
2. **link**：机器人的身体部分
3. **visual**：每个身体部分的外观
4. **geometry**：具体的形状
5. **material**：颜色和材质

## 形状类型

### 立方体（Box）
```xml
<box size="长 宽 高"/>
```

### 圆柱体（Cylinder）
```xml
<cylinder radius="半径" length="长度"/>
```

### 球体（Sphere）
```xml
<sphere radius="半径"/>
```

## 颜色表示

颜色用RGBA四个数字表示：
- R：红色（0-1）
- G：绿色（0-1）
- B：蓝色（0-1）
- A：透明度（0-1）

例如：`rgba="1 0 0 1"` 表示纯红色，不透明。

## 下一步

现在您理解了基本结构，接下来我们会学习：
- 如何用关节连接多个连杆
- 如何设置连杆的位置
- 如何让机器人动起来
