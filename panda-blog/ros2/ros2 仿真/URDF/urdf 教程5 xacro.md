# URDF基础教程 - 第五课：使用Xacro简化URDF

## 什么是Xacro？

**Xacro = XML Macros（XML宏）**

Xacro是ROS中用于简化URDF文件的工具，它允许您：
- **定义宏** - 重复使用的代码块
- **使用参数** - 可配置的变量
- **减少重复** - 让URDF更易维护

## 为什么需要Xacro？

### 传统URDF的问题
```text
传统URDF：每个零件都要写完整的代码
- 左前轮：完整的link定义
- 右前轮：完整的link定义（几乎相同）
- 左后轮：完整的link定义（几乎相同）
- 右后轮：完整的link定义（几乎相同）

结果：大量重复代码，难以维护
```

### Xacro的解决方案
```text
Xacro：定义一个宏，重复使用
- 定义轮子宏
- 调用4次，传入不同参数
- 结果：代码简洁，易于维护
```

## Xacro的基本语法

### 1. 文件扩展名
- **输入文件**：`.urdf.xacro`
- **输出文件**：`.urdf`
- **转换命令**：`xacro input.urdf.xacro > output.urdf`

### 2. 宏定义
```xml
<xacro:macro name="宏名称" params="参数1 参数2">
  <!-- 宏的内容 -->
</xacro:macro>
```

### 3. 宏调用
```xml
<xacro:宏名称 参数1="值1" 参数2="值2"/>
```

### 4. 参数使用
```xml
${参数名}
```

## 实际应用示例

### 示例1：最简单的宏

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">
  
  <!-- 定义盒子宏，添加位置参数 -->
  <xacro:macro name="box_link" params="name size x y z">
    <link name="${name}">
      <visual>
        <!-- 设置盒子的位置 -->
        <origin xyz="${x} ${y} ${z}" rpy="0 0 0"/>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>
  
  <!-- 使用宏创建两个盒子，设置不同的位置 -->
  <xacro:box_link name="盒子1" size="0.1 0.2 0.2" x="0.0" y="0.0" z="0.0"/>
  <xacro:box_link name="盒子2" size="0.1 0.1 0.1" x="0.2" y="0.0" z="0.0"/>
  
</robot>
```

**这个示例可以正常工作！**

### 测试步骤

1. **保存文件**：将代码保存为`simple.urdf.xacro`
2. **转换为URDF**：
   ```bash
   xacro simple.urdf.xacro > simple.urdf
   ```
3. **在RViz中查看**：
   ```bash
   ros2 launch urdf_tutorial display.launch.py model:=path/to/simple.urdf
   ```
4. **预期效果**：应该能看到两个红色的盒子

### 示例2：使用参数

```xml
<?xml version="1.0"?>
<robot name="参数机器人" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 定义参数 -->
  <xacro:property name="盒子大小" value="0.3"/>
  
  <!-- 定义材质 -->
  <material name="蓝色">
    <color rgba="0 0 1 1"/>
  </material>
  
  <!-- 使用参数 -->
  <link name="参数盒子">
    <visual>
      <geometry>
        <box size="${盒子大小} ${盒子大小} ${盒子大小}"/>
      </geometry>
      <material name="蓝色"/>
    </visual>
  </link>
  
</robot>
```

**这个示例也可以正常工作！**

### 测试步骤

1. **保存文件**：将代码保存为`params.urdf.xacro`
2. **转换为URDF**：
   ```bash
   xacro params.urdf.xacro > params.urdf
   ```
3. **在RViz中查看**：应该能看到一个蓝色的立方体
4. **修改参数**：将`盒子大小`改为0.5，重新转换查看效果

### 示例3：实用的轮子宏

```xml
<?xml version="1.0"?>
<robot name="四轮小车" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 定义材质 -->
  <material name="蓝色">
    <color rgba="0 0 1 1"/>
  </material>
  
  <material name="黑色">
    <color rgba="0 0 0 1"/>
  </material>
  
  <!-- 定义轮子宏 -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="0.1" radius="0.05"/>
        </geometry>
        <material name="黑色"/>
      </visual>
    </link>
    
    <joint name="${name}_关节" type="fixed">
      <parent link="车身"/>
      <child link="${name}"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 0"/>
    </joint>
  </xacro:macro>
  
  <!-- 车身 -->
  <link name="车身">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="蓝色"/>
    </visual>
  </link>
  
  <!-- 使用轮子宏 -->
  <xacro:wheel name="左前轮" x="0.2" y="0.15"/>
  <xacro:wheel name="右前轮" x="0.2" y="-0.15"/>
  <xacro:wheel name="左后轮" x="-0.2" y="0.15"/>
  <xacro:wheel name="右后轮" x="-0.2" y="-0.15"/>
  
</robot>
```

**这个示例创建完整的四轮小车！**

### 测试步骤

1. **保存文件**：将代码保存为`car.urdf.xacro`
2. **转换为URDF**：
   ```bash
   xacro car.urdf.xacro > car.urdf
   ```
3. **在RViz中查看**：应该能看到蓝色的车身和四个黑色的轮子

## 转换Xacro为URDF

### 命令行转换
```bash
# 基本转换
xacro robot.urdf.xacro > robot.urdf

# 指定输出文件
xacro robot.urdf.xacro -o robot.urdf
```

### 在Launch文件中使用
```xml
<launch>
  <!-- 直接使用xacro文件 -->
  <param name="robot_description" 
         command="$(find xacro)/xacro $(find 包名)/urdf/robot.urdf.xacro"/>
</launch>
```

## 常见错误

### 错误1：忘记xmlns声明
```xml
<!-- ❌ 错误：缺少xmlns声明 -->
<robot name="机器人">
  <xacro:macro name="test">...</xacro:macro>
</robot>
```

**正确写法**：
```xml
<robot name="机器人" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="test">...</xacro:macro>
</robot>
```

### 错误2：参数名不匹配
```xml
<!-- ❌ 错误：宏定义和调用参数不匹配 -->
<xacro:macro name="wheel" params="name x y">
  <!-- 宏内容 -->
</xacro:macro>

<xacro:wheel name="左前轮" x="0.2"/>  <!-- 缺少y参数 -->
```

## 总结

**Xacro的优势**：
- **减少重复** - 通过宏定义避免代码重复
- **易于维护** - 修改一处，全部更新
- **参数化** - 通过参数控制机器人配置

**使用要点**：
- **宏定义** - 使用`<xacro:macro>`定义重复代码
- **参数使用** - 使用`${参数名}`引用参数
- **转换验证** - 确保xacro能正确转换为urdf

**现在您可以：**
1. 使用宏简化重复代码
2. 使用参数控制机器人配置
3. 创建更易维护的URDF文件
