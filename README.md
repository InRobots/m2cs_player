# M2CS Player

[![README | 中文](https://img.shields.io/badge/README-%E4%B8%AD%E6%96%87-1f6feb?logo=readthedocs&logoColor=white)](./README_CN.md)

`m2cs_player` is a ROS1 + Qt5 offline multi-sensor playback tool. It can publish:

- GNSS/INS (`/gwnav/nav`, `/gwnav/gps`, `/gwnav/imu`)
- Livox left/right point clouds and IMU (`/livox_left/*`, `/livox_right/*`)
- Ouster point cloud and IMU (`/ouster/*`)
- Camera image (`/camera/image_raw`)
- Simulated clock (`/clock`)

It also provides a GUI for play/pause/loop/stop-skip/sensor toggles, and uses `m2cs_player.launch` to start RViz and `image_proc` debayer nodelet.

---

## 1. Project Info

- **ROS package name**: `m2cs_player`
- **Executable**: `m2cs_player`
- **Launch file**: `launch/m2cs_player.launch`
- **Maintainer**: Huanfeng Zhao (`zhaohf24@mails.jlu.edu.cn`)

---

## 2. Dependencies

Recommended environment: Ubuntu + ROS1 (catkin workspace).

### 2.1 ROS dependencies (declared in `package.xml`)

- `roscpp`, `rospy`, `std_msgs`, `geometry_msgs`
- `image_transport`, `cv_bridge`
- `pcl_ros`, `pcl_conversions`, `pcl_msgs`
- `camera_info_manager`, `tf`
- `cmake_modules`

### 2.2 Third-party dependencies (used in `CMakeLists.txt`)

- Eigen
- PCL
- OpenCV
- Qt5 (`Widgets`, `Gui`, `OpenGL`)

### 2.3 Extra message packages used in source code

The source includes these custom message packages. Make sure they are available in the same workspace:

- `gwnav_ros_driver`
- `livox_ros_driver`

---

## 3. Build

The commands below use your current workspace: `/home/awesome/macs_player_ws`.

First, clone the package into your workspace source folder (temporary URL placeholder):

```bash
cd /home/awesome/macs_player_ws/src
git clone https://example.com/temporary/m2cs_player.git
```

> Replace the placeholder URL above with your actual repository URL later.

```bash
cd /home/awesome/macs_player_ws
catkin_make
```

If you use `zsh`:

```bash
source /home/awesome/macs_player_ws/devel/setup.zsh
```

If you use `bash`:

```bash
source /home/awesome/macs_player_ws/devel/setup.bash
```

---

## 4. Run

```bash
source /home/awesome/macs_player_ws/devel/setup.zsh
roslaunch m2cs_player m2cs_player.launch
```

This launch file starts:

1. `m2cs_player` main application
2. `rviz` (config: `rviz/m2cs.rviz`)
3. `nodelet` + `image_proc/debayer` (namespace defaults to `camera`)

---

## 5. Dataset Directory Layout

After clicking **Load** in the GUI, the selected dataset root should contain at least:

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

> If path or files are invalid, the program prints: `Please check file path. Input path is wrong.`

---

## 6. GUI Usage

1. Click **Load** and select the dataset root folder.
2. Click **Play** to start playback (click again to end).
3. Click **Pause/Resume** to pause/resume.
4. Drag the slider to jump in timestamp progress.
5. Use checkboxes for options:
   - `Loop`: restart playback when finished
   - `Skip stop section`: skip stationary segments
   - `Auto start`: auto-play after loading
   - Sensor switches: `TF`, `Nav`, `Ouster`, `Left Livox`, `Right Livox`, `Camera`
6. Set `Speed` playback rate (`0.01 ~ 20.0`).

---

## 7. Troubleshooting

### 7.1 Build error: missing `gwnav_ros_driver` or `livox_ros_driver`

Put the required ROS packages in the same `catkin_ws/src` and rebuild:

```bash
cd /home/awesome/macs_player_ws
catkin_make
```

### 7.2 No image or point cloud during playback

- Verify dataset layout and filenames in Section 5
- Check whether corresponding sensor checkboxes are enabled
- Check RViz topics and fixed frame

---

## 8. Key Files

- UI: `src/mainwindow.ui`
- Main window logic: `src/mainwindow.cpp`
- Playback thread: `src/ROSThread.cpp`
- Launch file: `launch/m2cs_player.launch`

---

## Acknowledgements

- [Complex Urban Dataset File Player](https://github.com/RPM-Robotics-Lab/file_player_complex_urban.git)

---

## Citation

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
