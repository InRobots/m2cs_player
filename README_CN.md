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

## 2. 数据集下载

你可以通过以下链接下载数据集：

- [google](https://drive.google.com/drive/folders/1r73SmBBm7jIRvlrtqm2g_w5VX1Zy4SFH?usp=sharing)
- [baidu](https://pan.baidu.com/s/1iVqkIFc7AsaEojbos7S80w)（提取码：`m2cs`）

---

## 3. 环境依赖

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

## 4. 编译

建议按以下步骤在 catkin 工作空间中完成编译。

1. 创建工作空间（如已存在可跳过）：

```bash
mkdir -p ~/m2cs_player_ws/src
```

2. 在 `src` 目录克隆仓库：

```bash
cd ~/m2cs_player_ws/src
git clone https://github.com/InRobots/m2cs_player.git
```

3. 使用 catkin 编译：

```bash
cd ~/m2cs_player_ws
catkin_make
```

4. 加载工作空间环境：

如果你使用 `zsh`：

```bash
source ~/m2cs_player_ws/devel/setup.zsh
```

如果你使用 `bash`：

```bash
source ~/m2cs_player_ws/devel/setup.bash
```

---

## 5. 运行

```bash
roslaunch m2cs_player m2cs_player.launch
```

该 launch 会启动：

1. `m2cs_player` 主程序
2. `rviz`（配置文件：`rviz/m2cs.rviz`）
3. `nodelet` + `image_proc/debayer`（命名空间默认 `camera`）

---

## 6. 数据目录要求

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

## 7. GUI 使用说明

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

## 8. 常见问题

### 8.1 编译报找不到 `gwnav_ros_driver` 或 `livox_ros_driver`

将对应 ROS 包放到同一 `m2cs_player_ws/src` 下后重新编译。

### 8.2 运行后没有图像或点云

- 检查数据目录结构与文件名是否符合第 5 节
- 检查对应传感器复选框是否勾选
- 检查 RViz 话题与 fixed frame 配置

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
