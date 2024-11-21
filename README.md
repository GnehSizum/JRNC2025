# JRNC demo

这是为吉林大学 TARS_Go 战队 RoboMaster 校内赛智能导航与决策类赛题提供的仿真环境和示例程序。

## 1. 依赖

- ros-humble
- gazebo
    ```bash
    sudo apt install ros-humble-gazebo*
    ```
- navigation2

    ```bash
    sudo apt install ros-humble-navigation2
    ```

- slam-toolbox
  
    ```bash
    sudo apt install ros-humble-slam-toolbox
    ```

- behaviortree-cpp-v3

    ```bash
    sudo apt install ros-humble-behaviortree-cpp-v3
    ```

## 2. 目录结构

```
JRNC_SIM
├── behavior_tree  # 行为树实现
│   ├── CMakeLists.txt
│   ├── include
│   │   └── move_action.hpp  # Move行为节点
│   ├── package.xml
│   ├── src
│   │   └── main.cpp  # 主程序
│   └── tree
│       └── patrol.xml  # 行为树 XML 文件
├── demo_bringup  # 示例程序启动空间
│   ├── CMakeLists.txt
│   ├── config  # 参数目录
│   │   ├── mapper_params_online_async_sim.yaml  # slam-toolbox 建图参数文件
│   │   └── nav2_param.yaml  # navigation2 导航参数文件
│   ├── launch
│   │   ├── bringup.launch.py
│   │   └── mapping.launch.py  # 建图启动文件
│   ├── map
│   │   ├── jrnc.pgm  # 地图
│   │   └── jrnc.yaml
│   ├── package.xml
│   └── rviz
│       ├── mapping.rviz  # 建图模式 RViz 配置文件
│       └── nav2.rviz     # 导航模式 RViz 配置文件
└── simulation  # 仿真环境
    ├── CMakeLists.txt
    ├── launch
    │   └── sim_jrnc.launch.py  # 仿真环境启动文件
    ├── meshes  # 模型目录
    │   └── JRNC
    │       └── JRNC2025.dae  # 场地模型
    ├── package.xml
    ├── rviz
    │   └── 2d_lidar.rviz
    ├── urdf
    │   └── simple_robot.xacro  # 小车模型
    └── world
        ├── empty_world.world  # 空世界
        └── JRNC
            └── JRNC.world  # 场地环境
```

## 3. 即刻开玩

### 开始

创建一个新的工作空间：
```bash
mkdir -p your_ws/src
```

将 `behavior_tree` `demo_bringup` `simulation` 放入新建的 `src` 目录下。

### 构建

回到刚刚创建的工作空间目录下（your_ws）。

```bash
colcon build --symlink-install
```

### 启动 Gazebo 仿真

```bash
. install/setup.bash
ros2 launch simulation sim_jrnc.launch.py
```

### 建图

```bash
. install/setup.bash
ros2 launch demo_bringup mapping.launch.py 
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f your_path/map_file_name
```

### 导航

```bash
. install/setup.bash
ros2 launch demo_bringup bringup.launch.py 
```

### 巡逻

```bash
. install/setup.bash
ros2 run behavior_tree patrol_node
```