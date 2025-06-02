# 📊 盲道循迹避障视觉系统

一个基于OpenCV图像处理的盲道循迹避障视觉系统，适用于自动导航小车。该系统能够使用普通摄像头或ZED立体相机实时检测盲道，计算偏差，以及在遇到障碍物时自动规划避障路径。

## ✨ 功能特点

- 基于传统OpenCV图像处理，无需深度学习模型
- 支持普通USB摄像头或ZED立体相机
- 使用ZED相机时可基于深度信息进行智能避障
- 状态机架构的避障逻辑
- 盲道实时检测与偏差计算
- 基于轮廓分析或深度信息的障碍物检测
- 障碍物检测和自动避让
- 避障后回归循迹路线
- 串口通信控制接口
- 可视化调试界面

## 🏗️ 系统架构

系统由五种状态组成的状态机控制：

| 状态                | 描述                          |
| ------------------- | ----------------------------- |
| `FOLLOW_LINE`       | 盲道清晰 → 正常循迹          |
| `OBSTACLE_DETECTED` | 盲道被遮挡 → 检测障碍 → 决定避障方向 |
| `AVOIDING_LEFT`     | 向左执行避障动作（转向、直行、转回） |
| `AVOIDING_RIGHT`    | 向右执行避障动作（转向、直行、转回） |
| `LINE_REACQUIRED`   | 再次检测到盲道 → 回归循迹     |

主要组件：

1. **CameraCapture**: 管理摄像头和图像采集，支持标准相机和ZED相机
2. **LineDetector**: 处理图像，检测盲道和障碍物
3. **SerialCommunicator**: 向控制系统传输控制命令
4. **ConfigManager**: 管理JSON配置参数
5. **TrackVision**: 核心状态机逻辑和处理流程

## 🔧 软件依赖

- OpenCV 4.x
- C++11兼容的编译器
- CMake 3.10+
- 可选：ZED SDK 3.x（使用ZED相机时需要）
- 可选：CUDA（使用ZED相机时需要）

## 🛠️ 构建项目

### 在Ubuntu上构建:

```bash
# 安装必要的系统软件包
sudo apt update
sudo apt install build-essential cmake git libopencv-dev

# 克隆仓库并构建
git clone https://github.com/BreCaspian/blind-track-vision.git
cd blind-track-vision
mkdir build && cd build

# 仅使用标准相机构建
cmake ..
# 或者启用ZED相机支持构建
# cmake -DUSE_ZED_SDK=ON ..

make -j4
```

### 在Windows上构建:

1. 安装OpenCV（可使用预编译的二进制文件）
2. 安装CMake和Visual Studio
3. 设置OpenCV_DIR环境变量指向OpenCV构建目录
4. 如需ZED相机支持，安装ZED SDK和CUDA
5. 使用CMake生成Visual Studio项目文件：
   ```
   cmake -B build
   # 或者启用ZED相机支持
   # cmake -B build -DUSE_ZED_SDK=ON
   ```
6. 打开项目并构建

## 🚀 运行应用程序

```bash
# 从构建目录运行
./blind_track_vision

# 使用自定义配置文件
./blind_track_vision -c /path/to/config.json
```

### 📡 设置串口通信

#### Ubuntu系统：
1. 识别串口设备（通常为`/dev/ttyUSB0`或`/dev/ttyACM0`）：
   ```bash
   ls -l /dev/tty*
   ```

2. 设置用户权限：
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod a+rw /dev/ttyUSB0
   ```

3. 在config/vision_config.json中更新`serial.port`设置。

#### Windows系统：
- 从设备管理器中识别COM端口
- 在config/vision_config.json中相应更新`serial.port`设置

## ⚙️ 配置参数

系统使用JSON配置文件（`config/vision_config.json`），包含以下参数：

### 📷 相机设置
- `camera.id` - 相机ID（默认：0）
- `camera.width` - 图像宽度（默认：640）
- `camera.height` - 图像高度（默认：480）
- `camera.fps` - 目标帧率（默认：30）
- `camera.mode` - 相机模式（"standard"或"zed"，默认："standard"）
- `camera.depth_mode` - ZED相机深度模式（"PERFORMANCE"、"QUALITY"或"ULTRA"，默认："PERFORMANCE"）

### 🔍 盲道检测设置
- `detector.threshold` - 二值化阈值（默认：127）
- `detector.roi_y_start` - ROI起始位置占图像高度的比例（默认：0.6）
- `detector.roi_height` - ROI高度占图像高度的比例（默认：0.3）
- `detector.roi_width` - ROI宽度占图像宽度的比例（默认：0.5）
- `detector.canny_low` - Canny边缘检测低阈值（默认：50）
- `detector.canny_high` - Canny边缘检测高阈值（默认：150）
- `detector.hough_threshold` - Hough线检测阈值（默认：50）
- `detector.min_line_length` - 最小线段长度（默认：30）
- `detector.max_line_gap` - 最大线段间隙（默认：10）
- `detector.obstacle_contour_threshold` - 障碍物检测轮廓阈值（默认：5）

### 🚗 避障设置
- `avoidance.preferred_direction` - 优先避障方向（"right"或"left"，默认："right"）
- `avoidance.turn_time_ms` - 转弯时间（毫秒，默认：500）
- `avoidance.forward_time_ms` - 前进时间（毫秒，默认：800）
- `avoidance.check_interval_ms` - 检查间隔（毫秒，默认：100）
- `avoidance.min_line_detection_frames` - 确认回归所需的连续检测帧数（默认：3）
- `avoidance.obstacle_distance_threshold` - 障碍物检测距离阈值（米，默认：1.0）
- `avoidance.clear_path_threshold` - 判定路径通畅的距离阈值（米，默认：1.5）

### 🔌 串口通信设置
- `serial.port` - 串口名称（默认在Windows：`COM3`，Ubuntu：`/dev/ttyUSB0`）
- `serial.baudrate` - 串口波特率（默认：115200）

### 🐞 调试设置
- `debug.show_view` - 启用可视化调试窗口（默认：true）

## 📊 串口通信协议

系统使用二进制结构体通信协议，实现从视觉系统到电控系统的高效单向数据传输：

### 视觉系统 → 控制系统（VisionData结构体）（已实现）

```cpp
#pragma pack(push, 1)
struct VisionData {
    uint8_t header = 0xCD;         // 帧头
    float line_offset = 0.0f;      // 盲道偏差值（负值：左侧，正值：右侧）
    uint8_t line_found = 0;        // 盲道检测状态：0=未检测到，1=检测到
    uint8_t obstacle_detected = 0; // 障碍物检测状态：0=无障碍，1=有障碍
    uint8_t avoid_direction = 0;   // 避障方向：0=无，1=向左，2=向右
    float timestamp = 0.0f;        // 时间戳或帧编号
    uint8_t tail = 0xDC;           // 帧尾
};
#pragma pack(pop)
```

### 控制系统 → 视觉系统（Stm32Data结构体）（未实现）

```cpp
#pragma pack(push, 1)
struct Stm32Data {
    uint8_t header = 0xCD;       // 帧头
    uint8_t control_state = 0;   // 控制状态：0=循迹中，1=避障中，2=避障完成
    uint8_t motion_state = 0;    // 运动状态：0=停止，1=前进，2=左转，3=右转
    float timestamp = 0.0f;      // 时间戳，用于同步
    uint8_t tail = 0xDC;         // 帧尾
};
#pragma pack(pop)
```

**注意：** 当前实现仅使用单向通信（视觉→电控），协议中定义的接收功能（电控→视觉）未被启用。

## ❓ 故障排除

### 📷 相机问题
- 确保相机连接正常且驱动已正确安装
- 检查相机ID是否设置正确
- 尝试降低分辨率或帧率
- 使用ZED相机时，确保已安装ZED SDK和CUDA

### 🚧 检测问题
- 调整`detector.threshold`以适应不同光照条件
- 修改`detector.roi_*`参数以捕获图像中的正确区域
- 调整Canny和Hough参数以优化线路检测

### ⚠️ 避障问题
- 调整`avoidance.turn_time_ms`和`avoidance.forward_time_ms`以适应小车的实际速度
- 确保`detector.obstacle_contour_threshold`设置适合当前环境
- 使用ZED相机时，调整`avoidance.obstacle_distance_threshold`和`avoidance.clear_path_threshold`

## 👥 贡献

欢迎贡献！请随时提交Pull Request或报告问题。

## 📄 许可

MIT 许可证