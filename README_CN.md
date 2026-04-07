# M2CS Player

[![README | English](https://img.shields.io/badge/README-English-1f6feb?logo=readthedocs&logoColor=white)](./README.md)

`m2cs_player` 是一个基于 ROS1 + Qt5 的多传感器离线数据回放工具，支持以下数据发布：

- GNSS/INS（`/gwnav/nav`、`/gwnav/gps`、`/gwnav/imu`）
- Livox 左/右雷达点云与 IMU（`/livox_left/*`、`/livox_right/*`）
- Ouster 点云与 IMU（`/ouster/*`）
- 相机图像（`/camera/image_raw`）
- 仿真时钟（`/clock`）

同时提供 GUI 控制播放、暂停、循环、跳停与各传感器开关，并通过 `m2cs_player.launch` 联动 RViz 与 `image_proc` 去马赛克节点。

---

## 1. 工程信息

- **ROS 包名**：`m2cs_player`
- **可执行程序**：`m2cs_player`
- **启动文件**：`launch/m2cs_player.launch`
- **维护者**：Huanfeng Zhao (`zhaohf24@mails.jlu.edu.cn`)

---

## 2. 环境依赖

建议环境：Ubuntu + ROS1（catkin 工作空间）。

### 2.1 ROS 依赖（`package.xml` 中声明）

- `roscpp`、`rospy`、`std_msgs`、`geometry_msgs`
- `image_transport`、`cv_bridge`
- `pcl_ros`、`pcl_conversions`、`pcl_msgs`
- `camera_info_manager`、`tf`
- `cmake_modules`

### 2.2 第三方依赖（`CMakeLists.txt` 中使用）

- Eigen
- PCL
- OpenCV
- Qt5（`Widgets`、`Gui`、`OpenGL`）

### 2.3 额外消息包（源码中包含头文件）

源码使用了以下自定义消息类型，请确保它们在同一工作空间可见：

- `gwnav_ros_driver`
- `livox_ros_driver`

---

## 3. 编译

以下示例基于你当前工作空间：`/home/awesome/macs_player_ws`。

先将仓库克隆到工作空间的 `src` 目录（临时链接占位）：

```bash
cd /home/awesome/macs_player_ws/src
git clone https://example.com/temporary/m2cs_player.git
```

> 后续请将上面的占位链接替换为你的真实仓库地址。

```bash
cd /home/awesome/macs_player_ws
catkin_make
```

如果你使用 `zsh`：

```bash
source /home/awesome/macs_player_ws/devel/setup.zsh
```

如果你使用 `bash`：

```bash
source /home/awesome/macs_player_ws/devel/setup.bash
```

---

## 4. 运行

```bash
source /home/awesome/macs_player_ws/devel/setup.zsh
roslaunch m2cs_player m2cs_player.launch
```

该 launch 会启动：

1. `m2cs_player` 主程序
2. `rviz`（配置文件：`rviz/m2cs.rviz`）
3. `nodelet` + `image_proc/debayer`（命名空间默认 `camera`）

---

## 5. 数据目录要求

在 GUI 中点击 **Load** 后，选择的数据根目录需包含至少以下内容：

```text
<dataset_root>/
├── data_stamp.csv
├── ins_data/
│   ├── gwnav_nav.csv
│   ├── gwnav_imu.csv
│   ├── gwnav_gps.csv
│   ├── livox_left_imu.csv
│   ├── livox_right_imu.csv
│   └── ouster_imu.csv
├── calib_data/
│   ├── livox_left_to_ouster.yaml
│   ├── livox_right_to_ouster.yaml
│   ├── camera_to_ouster.yaml
│   ├── ouster_to_gwnav.yaml
│   └── camera.yaml
├── lidar_data/
│   ├── livox_left/*.bin
│   ├── livox_right/*.bin
│   └── ouster/*.bin
└── image_data/*.png
```

> 若路径错误或文件缺失，程序会在终端提示：`Please check file path. Input path is wrong.`

---

## 6. GUI 使用说明

1. 点击 **Load** 选择数据目录。
2. 点击 **Play** 开始回放（再次点击可结束）。
3. 点击 **Pause/Resume** 暂停或恢复。
4. 通过滑条跳转时间戳进度。
5. 通过复选框控制功能：
	- `Loop`：回放到末尾后循环
	- `Skip stop section`：跳过静止区间
	- `Auto start`：加载完成后自动开始
	- 传感器开关：`TF`、`Nav`、`Ouster`、`Left Livox`、`Right Livox`、`Camera`
6. `Speed` 可调播放倍率（`0.01 ~ 20.0`）。

---

## 7. 常见问题

### 7.1 编译报找不到 `gwnav_ros_driver` 或 `livox_ros_driver`

将对应 ROS 包放到同一 `catkin_ws/src` 下后重新编译：

```bash
cd /home/awesome/macs_player_ws
catkin_make
```

### 7.2 运行后没有图像或点云

- 检查数据目录结构与文件名是否符合第 5 节
- 检查对应传感器复选框是否勾选
- 检查 RViz 话题与 fixed frame 配置

---

## 8. 相关文件

- UI：`src/mainwindow.ui`
- 主窗口逻辑：`src/mainwindow.cpp`
- 数据回放线程：`src/ROSThread.cpp`
- 启动文件：`launch/m2cs_player.launch`

---

## 致谢

- [Complex Urban Dataset File Player](https://github.com/RPM-Robotics-Lab/file_player_complex_urban.git)

---

## 引用

```bibtex
@article{zhao2025m2cs,
	title={M2CS: A Multimodal and Campus-Scapes Dataset for Dynamic SLAM and Moving Object Perception},
	author={Zhao, Huanfeng and Yao, Meibao and Zhao, Yan and Jiang, Yao and Zhang, Hongyan and Xiao, Xueming and Gao, Ke},
	journal={Journal of Field Robotics},
	volume={42},
	number={3},
	pages={787--805},
	year={2025},
	publisher={Wiley Online Library}
}
```
