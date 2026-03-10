Laser Angle Filter (ROS)

一个基于 ROS 的轻量级雷达角度过滤器，支持双扇区过滤以及动态调参 (Dynamic Reconfigure)。特别针对需要跨越 0 度线（例如 350° 到 10°）的过滤场景进行了逻辑优化。 

## 1. 功能特性

    0-360° 逻辑：自动将雷达原始角度归一化到 0-360° 范围进行处理。

    跨零点支持：支持设置如 min: 350, max: 10 的过滤区间，完美解决 0 度线跳变问题。

    实时调试：集成 dynamic_reconfigure，无需重启节点即可通过 GUI 调整过滤范围。 

多区间过滤：内置两组独立的角度过滤区间（Filter 1 & Filter 2）。 

## 2. 环境依赖

    操作系统: Ubuntu 20.04 (推荐)

    ROS 版本: Noetic (或兼容 dynamic_reconfigure 的其他版本)

    核心依赖: roscpp, sensor_msgs, dynamic_reconfigure

## 3. 安装步骤
```Bash

# 进入你的工作空间
cd ~/catkin_ws/src

# 克隆仓库 (请替换为你实际的 GitHub 链接)
git clone https://github.com/Pro-qing/laser_angle_filter.git

# 编译
cd ..
catkin_make
source devel/setup.bash
```
## 4. 快速使用

你可以直接运行预设的 launch 文件，它会同时启动过滤器节点、rqt 可视化调参工具以及 RViz。 

```Bash

roslaunch laser_angle_filter filter.launch

默认配置参数 

参数名	默认值	描述
scan_in_topic	/scan_right	输入的原始雷达话题
scan_out_topic	/scan_filtered	过滤后的雷达话题
```

## 5. 动态调参说明 

启动节点后，在弹出的 rqt_reconfigure 窗口中可以调整以下参数：

    angle1_min / max: 第一组过滤区间的起始与结束角度 (0-360°)。

    angle2_min / max: 第二组过滤区间的起始与结束角度 (0-360°)。

    注意：如果 min > max（例如 min=350, max=20），程序会自动识别为跨越 0 度线的区域进行过滤。

## 6. 项目结构

cfg/LaserFilter.cfg: 定义动态调参的参数结构。 

launch/filter.launch: 节点启动配置。 

src/laser_filter_node.cpp: 核心过滤逻辑实现。

rviz/laser_filter.rviz: 预设的 RViz 视图配置文件。 

Author: Pro-qing