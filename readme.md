# fast_ws

基于 ROS Noetic 的 **Fast Drone 250** 感知与控制工作空间。集成视觉惯性里程计（VINS-Fusion）、局部避障规划（EGO-Planner）、PX4 飞控接口（px4ctrl），并在 RK3588 NPU 上运行 RKNN 推理，支持 **自主导航** 与 **人体跟随** 两种飞行模式。

---

## 功能概览

| 模块 | 包名 | 说明 |
|------|------|------|
| 视觉惯性里程计 | `vins` | 双目红外 + IMU 融合定位，输出 `/vins_fusion/imu_propagate` |
| 深度估计 | `ego_depth_fused_node` | MiDaS RKNN 单目深度 + VINS 尺度融合，输出 `/ego/depth_image` |
| 轨迹规划 | `ego_planner` | 基于深度图的局部避障规划，输出 B 样条轨迹 |
| 飞控接口 | `px4ctrl` | 接收 `/position_cmd`，通过 MAVROS 控制 PX4 |
| 人体跟随 | `human_follow_rknn` | YOLOv5 RKNN 检测 + CSRT 跟踪，生成跟随指令 |

---

## 系统架构

### 自主导航模式

```
RealSense (双目 IR + 深度)
        │
        ├──► VINS-Fusion ──► /vins_fusion/imu_propagate ──┐
        │                                                  │
        └──► (可选) ego_depth_fused_node ──► /ego/depth_image
                                                           │
                                              EGO-Planner ◄┘
                                                   │
                                            /position_cmd
                                                   │
                                              px4ctrl ──► MAVROS ──► PX4
```

### 人体跟随模式

```
USB 相机
    │
    ▼
human_tracker_node (RKNN YOLO + CSRT)
    │  person_target (TargetInfo)
    ▼
human_follow_px4ctrl_node
    │  /position_cmd
    ▼
px4ctrl ──► MAVROS ──► PX4
```

> 人体跟随模式仍需要 VINS 提供里程计（`px4ctrl` 订阅 `/vins_fusion/imu_propagate`），但跟踪使用独立 USB 相机，不依赖 RealSense 可见光流。

---

## 目录结构

```
fast_ws/
├── model/                          # RKNN 模型文件
│   ├── yolov5n_rk3588.rknn         # 人体检测
│   └── midas_v21_small_256.rknn    # 单目深度估计
├── shfiles/                        # 常用启动脚本
│   ├── make.sh                     # 编译工作空间
│   ├── sys.sh                      # 系统性能模式 + 串口权限
│   ├── rspx4.sh                    # 启动 RealSense / MAVROS / VINS
│   ├── takeoff.sh                  # 一键起飞
│   ├── land.sh                     # 一键降落
│   ├── record.sh                   # rosbag 录制
│   └── checkcpu.sh                 # 查看 CPU 频率
└── src/
    ├── VINS-Fusion/                # 视觉惯性里程计
    ├── px4ctrl/                    # PX4 控制接口
    ├── planner/                    # EGO-Planner 及依赖
    ├── human_follow_rknn/          # 人体跟随（自研）
    └── utils/                      # 公共工具包
```

---

## 硬件要求

| 组件 | 说明 |
|------|------|
| 机载计算机 | RK3588（需安装 RKNN Runtime，`librknnrt.so`） |
| 飞控 | PX4 固件，通过 USB 串口连接（默认 `/dev/ttyACM0`） |
| 相机（导航） | Intel RealSense（D435/D435i 等），提供双目红外与深度 |
| 相机（跟随） | USB 摄像头（`/dev/video0`，640×480） |
| 遥控器 | 建议保留 RC 作为安全接管手段 |

---

## 软件依赖

- **操作系统**：Ubuntu 20.04 + ROS Noetic
- **ROS 包**：`mavros`、`realsense2_camera`、`cv_bridge`、`image_transport` 等
- **第三方库**：OpenCV 4、Ceres Solver、Eigen3
- **RKNN**：Rockchip RKNN Toolkit2 运行时（`rknn_api.h`、`librknnrt.so`）

安装 ROS 依赖示例：

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras \
                 ros-noetic-realsense2-camera \
                 ros-noetic-cv-bridge ros-noetic-image-transport
```

---

## 编译

```bash
cd ~/fast_ws
source /opt/ros/noetic/setup.bash

# 或使用项目脚本
bash shfiles/make.sh
```

`make.sh` 内容：

```bash
catkin_make --pkg quadrotor_msgs -DCMAKE_BUILD_TYPE=Release
catkin_make -DCMAKE_BUILD_TYPE=Release
```

编译完成后：

```bash
source devel/setup.bash
```

> **注意**：`human_follow_rknn` 和 `ego_depth_fused_node` 依赖 RKNN 库。若头文件或库路径非默认位置，请修改对应 `CMakeLists.txt` 中的 `include_directories` 与 `link_directories`。

---

## 快速启动

### 1. 系统准备

```bash
bash shfiles/sys.sh        # CPU 性能模式 + 串口权限
bash shfiles/checkcpu.sh   # 确认 CPU 频率
```

### 2. 感知与通信（自主导航）

```bash
bash shfiles/rspx4.sh
# 等效于依次启动：
#   roslaunch realsense2_camera rs_camera.launch
#   roslaunch mavros px4.launch
#   roslaunch vins fast_drone_250.launch
```

VINS 配置文件：`src/VINS-Fusion/config/fast_drone_250.yaml`

- IMU 话题：`/mavros/imu/data_raw`
- 左/右目：`/camera/infra1/image_rect_raw`、`/camera/infra2/image_rect_raw`

### 3. 飞控控制

```bash
roslaunch px4ctrl run_ctrl.launch
```

关键 remap（见 `run_ctrl.launch`）：

- 里程计：`/vins_fusion/imu_propagate`
- 位置指令：`/position_cmd`

控制参数：`src/px4ctrl/config/ctrl_param_fpv.yaml`（质量、推力模型、PID 增益等，**部署前务必标定**）。

### 4. 轨迹规划（EGO-Planner）

**实机 + RealSense 深度：**

```bash
roslaunch ego_planner single_run_in_exp.launch
```

**VINS + RKNN 单目深度（无 RealSense 深度时）：**

```bash
roslaunch ego_planner single_run_vins_depth.launch.launch
```

在 RViz 中使用 **2D Nav Goal** 设置目标点（`flight_type=1`），或修改 launch 中的全局航点（`flight_type=2`）。

### 5. 起飞 / 降落

```bash
bash shfiles/takeoff.sh   # 发布 TakeoffLand takeoff_land_cmd: 1
bash shfiles/land.sh      # 发布 TakeoffLand takeoff_land_cmd: 2
```

---

## 人体跟随

### 启动

```bash
# 终端 1：感知 + 通信 + VINS + px4ctrl（同上）
bash shfiles/rspx4.sh
roslaunch px4ctrl run_ctrl.launch

# 终端 2：人体跟随
roslaunch human_follow_rknn human_follow_with_px4ctrl.launch
```

### 操作流程

1. 在 `tracker_view` 窗口按 **`S`** 框选目标人物（ROI）
2. 跟踪器使用 **CSRT** 持续跟踪；目标丢失超过 `miss_patience` 帧后，自动调用 **RKNN YOLOv5** 重新检测并恢复跟踪
3. `human_follow_px4ctrl_node` 根据目标在图像中的位置与 bbox 高度，生成 `/position_cmd` 控制无人机跟随
4. 按 **`Q`** 退出跟踪窗口

### 仅调试跟踪（不飞）

```bash
roslaunch human_follow_rknn debug_tracker.launch
```

### 主要参数

| 节点 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| human_tracker | `model_path` | `model/yolov5n_rk3588.rknn` | YOLO RKNN 模型路径 |
| human_tracker | `miss_patience` | 20 | 丢失容忍帧数 |
| human_follow_ctrl | `follow_height` | 3.0 m | 跟随高度 |
| human_follow_ctrl | `desired_h_norm` | 0.3 | 期望 bbox 高度占比（控制前后距离） |
| human_follow_ctrl | `k_yaw` / `k_v` | 0.8 / 2.0 | 偏航与前进增益 |
| human_follow_ctrl | `v_max` | 2.0 m/s | 最大水平速度 |
| human_follow_ctrl | `xy_bound` | 10.0 m | 相对起飞点最大水平偏移 |

---

## 主要 ROS 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vins_fusion/imu_propagate` | `nav_msgs/Odometry` | VINS 高频里程计（px4ctrl 使用） |
| `/vins_fusion/odometry` | `nav_msgs/Odometry` | VINS 优化后里程计 |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | RealSense 左目红外 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | RealSense 深度（EGO 实机模式） |
| `/ego/depth_image` | `sensor_msgs/Image` | RKNN 融合深度（EGO 单目模式） |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 位置控制指令 |
| `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | 起飞 / 降落命令 |
| `/human_tracker/person_target` | `human_follow_rknn/TargetInfo` | 人体跟踪结果 |

---

## 数据录制

```bash
bash shfiles/record.sh
```

默认录制 VINS 轨迹、规划可视化、深度图、控制指令等话题，便于离线分析。

---

## 配置说明

### VINS 外参与标定

- 相机内参：`src/VINS-Fusion/config/cam_imu/left.yaml`、`right.yaml`
- IMU-相机外参：`src/VINS-Fusion/config/fast_drone_250.yaml` 中 `body_T_cam0/1`
- 首次部署建议重新标定并更新上述文件

### px4ctrl 推力标定

```bash
roslaunch px4ctrl thrust_calibrate.launch
```

标定完成后更新 `ctrl_param_fpv.yaml` 中的 `thrust_model` 或 `hover_percentage`。

### RKNN 模型

模型位于 `model/` 目录，launch 文件中通过相对路径引用：

```
$(find human_follow_rknn)/../../model/yolov5n_rk3588.rknn
```

`ego_depth_fused_node` 默认模型路径为 `midas_v21_small_256.rknn`，可通过 ROS 参数 `model_path` 覆盖。

---

## 常见问题

**Q：编译报错找不到 `rknn_api.h` 或 `librknnrt.so`**

确认 RKNN Runtime 已安装，并修改 `human_follow_rknn/CMakeLists.txt` 中的 include / lib 路径。

**Q：VINS 无法初始化**

- 检查 RealSense 红外图像是否正常发布
- 确认 IMU 话题 `/mavros/imu/data_raw` 有数据
- 起飞前保持静止，给予足够激励完成初始化

**Q：px4ctrl 不响应指令**

- 确认已订阅到 `/vins_fusion/imu_propagate`
- 检查 RC 是否连接（`no_RC: false` 时需要遥控器）
- 确认 MAVROS 与 PX4 通信正常（`mavros/state` 中 `connected: true`）

**Q：人体跟随目标频繁丢失**

- 适当增大 `miss_patience`
- 确保光照充足、目标占据足够像素
- 检查 USB 相机帧率是否稳定

---

## 致谢与参考

本项目基于以下开源工作集成与扩展：

- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) — 视觉惯性里程计
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) — 局部轨迹规划
- [PX4Ctrl](https://github.com/ZJU-FAST-Lab/px4ctrl) — PX4 控制框架
- [RKNN Model Zoo](https://github.com/airockchip/rknn_model_zoo) — RK3588 NPU 推理

---

## 许可证

各子模块遵循其原始许可证（如 px4ctrl 为 GPLv3、human_follow_rknn 为 MIT）。使用前请阅读对应目录下的 LICENSE 文件。
