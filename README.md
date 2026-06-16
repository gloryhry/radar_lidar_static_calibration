# radar_lidar_static_calibration

[![Actions Status](https://github.com/gloryhry/radar_lidar_static_calibration/workflows/CI/badge.svg)](https://github.com/gloryhry/radar_lidar_static_calibration)
[![GitHub](https://img.shields.io/github/license/gloryhry/radar_lidar_static_calibration)](https://github.com/gloryhry/radar_lidar_static_calibration/blob/master/LICENSE)

雷达(Radar)与激光雷达(LiDAR)之间的**静态外参标定**工具。本项目是一个 ROS1 catkin 软件包，提供了一套完整的标定工作流程：从数据采集、点云匹配、手动选点，到最终计算雷达与激光雷达之间的外参变换矩阵。

---

## 目录结构

```
radar_lidar_static_calibration/
├── CMakeLists.txt                     # CMake 编译配置
├── package.xml                        # 软件包元信息与依赖
├── LICENSE                            # GPLv3 许可证
├── README.md
├── install_dependencies.sh            # 系统依赖一键安装脚本
├── data/
│   └── calibration.csv                # radar_match 输出的标定结果
├── include/
│   ├── calibration.h                  # RadarCalibration 类声明
│   ├── radar_lidar_calibration.h      # CalibrationPair 类及伪逆运算
│   └── radar_match/
│       └── match.h                    # RadarMatch 类声明
├── launch/
│   ├── calibration.launch             # 数据采集节点启动文件
│   └── match.launch                   # 帧间匹配节点启动文件
├── src/
│   ├── calibration.cpp                # RadarCalibration 类实现
│   ├── radar_lidar_calibration.cpp    # calibration 可执行文件入口
│   ├── radar_lidar_static_calibration_node.cpp  # 数据采集节点入口
│   ├── select_point.cpp               # 点云选点工具入口
│   └── radar_match/
│       ├── radar_match.cpp            # radar_match 可执行文件入口
│       └── match.cpp                  # RadarMatch 类实现（ICP/NDT匹配）
└── .github/
    └── workflows/
        └── main.yml                   # CI 配置(GitHub Actions)
```

---

## 依赖

### 系统依赖

| 依赖 | 说明 |
|------|------|
| C++17 | 编译器支持 |
| OpenCV | 点云可视化与图像处理 |
| PCL (Point Cloud Library) | 点云数据处理与可视化 |
| Eigen3 | 线性代数运算 |
| Boost | 线程与互斥 |

### ROS 依赖

| 依赖 | 说明 |
|------|------|
| roscpp / rospy | ROS 核心库 |
| cv_bridge | ROS 与 OpenCV 桥接 |
| message_filters | 多话题时间同步 |
| pcl_ros / pcl_conversions | ROS 与 PCL 桥接 |
| sensor_msgs | 传感器消息类型 (PointCloud2) |
| nav_msgs | 导航消息类型 (Odometry) |
| tf | 坐标变换 |
| visualization_msgs | 可视化消息 |

---

## 编译

### 1. 安装系统依赖

```bash
# 运行安装脚本（自动检测 ROS 版本并安装对应包）
cd <your_catkin_ws>/src/radar_lidar_static_calibration
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### 2. 克隆并编译

```bash
# 创建 catkin 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆本项目
git clone https://github.com/gloryhry/radar_lidar_static_calibration.git

cd ~/catkin_ws

# 编译
catkin init
catkin build --summarize --no-status --force-color
source devel/setup.bash
```

> **支持的 ROS 发行版**: Kinetic, Melodic

---

## 可执行文件说明

本项目共构建 **4 个可执行文件**和 **1 个共享库**。

---

### 1. `radar_lidar_static_calibration_node` — 数据采集节点

**功能**: 同步订阅雷达和激光雷达的 `PointCloud2` 话题，通过消息过滤器实现近似时间戳对齐。**按下空格键**时，自动将当前配对的雷达和激光雷达点云保存为 PCD 文件到 `data/` 目录。

**源代码**: `src/radar_lidar_static_calibration_node.cpp`

**使用方法**:

```bash
# 方式一：使用 launch 文件启动
roslaunch radar_lidar_static_calibration calibration.launch

# 方式二：直接运行并指定话题参数
rosrun radar_lidar_static_calibration radar_lidar_static_calibration_node \
    _lidar_topic:=/lidar/vlp16_back/PointCloud2 \
    _radar_topic:=/RadarDetectionImageNear_0
```

**操作步骤**:
1. 启动节点后，确保雷达和激光雷达数据正常发布
2. 将标定物（如角反射器）放置在雷达和激光雷达均能观测到的位置
3. 在终端中**按下空格键 (Space)**，节点会保存当前配对的 PCD 文件
4. 多次在不同位置摆放标定物，重复上述操作以收集多组数据
5. 数据文件将保存在 `data/radar_<timestamp>.pcd` 和 `data/lidar_<timestamp>.pcd`

**ROS 参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `lidar_topic` | string | `/lidar_PointCloud2` | 激光雷达话题名 |
| `radar_topic` | string | `/radar_PointCloud2` | 雷达话题名 |
| `sync_queue_size` | int | `500` | 时间同步队列大小 |

---

### 2. `calibration` — 标定计算节点

**功能**: 加载 `data/` 目录下的 PCD 文件对（雷达 + 激光雷达），通过 PCL 可视化窗口让用户**手动点选对应点对**，然后使用**最小二乘法（伪逆矩阵）**计算雷达坐标系到激光雷达坐标系的仿射变换参数（旋转 + 平移）。

**源代码**: `src/radar_lidar_calibration.cpp`

**使用方法**:

```bash
rosrun radar_lidar_static_calibration calibration \
    _pre_trans_x:=0.0 \
    _pre_trans_y:=0.0 \
    _pre_trans_theta:=1.57
```

**操作步骤**:

1. 确保已完成数据采集（`data/` 目录中有配对的 `radar_*.pcd` 和 `lidar_*.pcd` 文件）
2. 启动 `calibration` 节点，PCL 可视化窗口将打开
3. 窗口中将显示雷达点云（绿色）和激光雷达点云（白色）
4. 按住 **Shift + 鼠标左键** 在点云上选取对应点——**先点雷达点云上的点，再点激光雷达点云上对应的点**（每个点云各选一个点，共2个点组成一对）
5. 选好一对后按 **Q 键**进入下一组数据
6. 对所有数据重复步骤 4-5
7. 所有组数据标记完成后，程序将自动在终端输出计算得到的**仿射变换参数** `[a1, a2, a3, a4, a5, a6]`

**ROS 参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pre_trans_x` | double | `0.0` | 预变换 x 方向平移 (米) |
| `pre_trans_y` | double | `0.0` | 预变换 y 方向平移 (米) |
| `pre_trans_theta` | double | `1.57` | 预变换旋转角度 (弧度) |

> **预变换的作用**: 在手动选点之前，先对雷达点云进行一次粗略的初始变换（旋转 + 平移），使雷达和激光雷达点云在空间中大致对齐，便于手动精确选点。

**算法原理**:

标定采用仿射变换模型，设雷达坐标 `(x, y)` 映射到激光雷达坐标 `(u, v)`：

```
u = a1 * x + a2 * y + a3
v = a4 * x + a5 * y + a6
```

将多组对应点组成超定方程组，使用**伪逆矩阵**（最小二乘解）求解最优参数：

```
param = (M^T * M)^{-1} * M^T * w
```

其中 `M` 为雷达点坐标矩阵，`w` 为激光雷达点坐标向量。

---

### 3. `radar_match` — 雷达帧间匹配节点

**功能**: 通过 **ICP (Iterative Closest Point)** 算法匹配连续两帧雷达点云，同时与里程计 (Odometry) 数据进行对比，计算 **雷达到 IMU/里程计坐标系的外参变换**（即 `Odom2Radar` 变换）。结果输出到 `data/calibration.csv`。可视化窗口分为左右两栏：左侧显示匹配前的两帧点云，右侧显示 ICP 匹配后的结果。

**源代码**: `src/radar_match/radar_match.cpp` + `src/radar_match/match.cpp`

**使用方法**:

```bash
# 方式一：使用 launch 文件启动
roslaunch radar_lidar_static_calibration match.launch

# 方式二：直接运行并指定话题参数
rosrun radar_lidar_static_calibration radar_match \
    _radar_topic:=/RadarDetection/Pointcloud2/Near_0 \
    _Odometry_topic:=/localization/ackermanpos
```

**操作步骤**:
1. 确保车辆（或机器人）在**低速运动**，且雷达和里程计数据正常发布
2. 启动 `radar_match` 节点，PCL 可视化窗口将打开
3. 左右两侧窗口将实时显示：
   - **左侧 (Unregistered)**: 白色 = 当前帧点云，黄色 = 上一帧点云
   - **右侧 (Registered)**: 白色 = 当前帧点云，绿色 = ICP 匹配对齐后的点云
4. 当车辆旋转超过 **2°** 时，程序自动将当前计算的外参写入 `data/calibration.csv`
5. 累积足够数据后，关闭程序，在 `calibration.csv` 中查看标定结果

**ROS 参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `radar_topic` | string | `/radar` | 雷达话题名 |
| `Odometry_topic` | string | `/localization/ackermanekf` | 里程计话题名 |
| `sync_queue_size` | int | `50` | 时间同步队列大小 |

**ICP 匹配参数** (在 `match.cpp` 中配置):

| 参数 | 值 | 说明 |
|------|------|------|
| 最大对应距离 | `20` | 超过此距离的对应点对被忽略 |
| 最大迭代次数 | `100` | ICP 收敛的最大迭代次数 |
| 变换 epsilon | `0.001` | 收敛判定阈值 |
| 欧氏距离适应度 epsilon | `0.01` | 适应度收敛判定阈值 |

> **注意**: 代码中同时提供了 NDT (Normal Distributions Transform) 匹配方法的实现（已注释），如需使用可取消注释相应的代码段。

**输出格式** (`data/calibration.csv`):

```csv
<IMU2Radar_angle>,<IMU2Radar_x>,<IMU2Radar_y>
```

每行一个标定样本，分别为雷达到里程计坐标系的角度偏移（弧度）和 x、y 方向平移（米）。

---

### 4. `select_point` — 点云选点工具

**功能**: 辅助调试工具。加载指定的 PCD 文件，通过 PCL 可视化窗口让用户**手动选择点云中的 3D 点**，并在终端输出所选点的坐标。

**源代码**: `src/select_point.cpp`

**使用方法**:

```bash
rosrun radar_lidar_static_calibration select_point
```

> **注意**: 当前版本中 PCD 文件路径是硬编码的。如需修改，请编辑 `src/select_point.cpp` 第 69 行中的文件路径。

**操作步骤**:
1. 启动程序，PCL 可视化窗口将打开并显示点云
2. 按 **Shift + 鼠标左键** 点选目标点，终端将打印该点坐标 `(x, y, z)`
3. 按 **Q 键**退出

---

### 共享库: `libradar_calibration`

**功能**: 将雷达和激光雷达点云投影到鸟瞰图 (BEV) 图像上，转换为 OpenCV 图像格式（`cv::Mat`）供可视化使用。

**参数**: 分辨率为 `0.05` 米/像素，图像尺寸为 `60m × 60m`（即 `1200 × 1200` 像素）。

---

## 标定工作流程

以下是完整的端到端标定流程：

```
┌─────────────────────────────────────────────────────────┐
│                    两种标定方案                           │
├───────────────────────┬─────────────────────────────────┤
│  方案一: 手动标定法     │  方案二: 自动匹配法              │
│  (calibration)        │  (radar_match)                  │
├───────────────────────┼─────────────────────────────────┤
│  1. 部署标定物         │  1. 车辆低速转弯行驶             │
│  2. 数据采集并保存PCD  │  2. 同步记录雷达+里程计数据      │
│  3. 手动点选对应点对   │  3. ICP帧间匹配                 │
│  4. 最小二乘计算外参   │  4. 输出到CSV文件                │
│  5. 输出仿射变换参数   │  5. 统计分析外参                 │
└───────────────────────┴─────────────────────────────────┘
```

### 方案一: 手动标定法（静态场景，精度高，推荐）

1. **准备**: 将角反射器（雷达反射器）放置在车辆前方，确保雷达和激光雷达均能看到
2. **数据采集**: 运行 `radar_lidar_static_calibration_node`，按空格键保存点云
3. **多点采集**: 将标定物移动 5-10 个不同位置，重复步骤 2
4. **手动匹配**: 运行 `calibration`，为每对点云手动选择对应点（Shift+点击）
5. **输出外参**: 程序自动最小二乘法计算并输出标定参数

### 方案二: 自动匹配法（动态场景）

1. **准备**: 在空旷场地低速行驶，同时录制雷达和里程计数据
2. **运行匹配**: 运行 `radar_match` 节点，实时匹配雷达帧并计算外参
3. **采集数据**: 当车辆旋转角度超过2°时，自动写入 `calibration.csv`
4. **分析与平均**: 对多次标定结果取平均，得到最终外参

---

## 许可证

本项目基于 [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html) 开源。

---

## 作者

- **Glory Huang**
- 主页: [https://xjtuglory.ml](https://xjtuglory.ml)
- GitHub: [https://github.com/gloryhry](https://github.com/gloryhry)
