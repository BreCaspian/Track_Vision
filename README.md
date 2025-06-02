# 📊 Blind Track Vision System

A blind track vision system based on OpenCV image processing, suitable for automatic guided vehicles. The system can detect blind guide tracks, calculate deviation, and automatically plan obstacle avoidance paths using standard USB cameras or ZED stereo cameras.

[中文文档](README_CN.md)

## ✨ Features

- Traditional OpenCV image processing without deep learning models
- Support for standard USB cameras or ZED stereo cameras
- Depth-based intelligent obstacle avoidance when using ZED cameras
- State machine architecture for obstacle avoidance logic
- Real-time blind track detection and deviation calculation
- Obstacle detection based on contour analysis or depth information
- Automatic obstacle detection and avoidance
- Return to track following after obstacles are cleared
- Serial communication control interface
- Visual debugging interface

## 🏗️ System Architecture

The system is controlled by a state machine with five states:

| State               | Description                         |
| ------------------- | ----------------------------------- |
| `FOLLOW_LINE`       | Clear track → Normal tracking       |
| `OBSTACLE_DETECTED` | Track blocked → Detect obstacle → Determine avoidance direction |
| `AVOIDING_LEFT`     | Execute left avoidance maneuver (turn, move forward, turn back) |
| `AVOIDING_RIGHT`    | Execute right avoidance maneuver (turn, move forward, turn back) |
| `LINE_REACQUIRED`   | Track detected again → Return to normal tracking |

Main components:

1. **CameraCapture**: Manages camera and image acquisition, supports standard cameras and ZED cameras
2. **LineDetector**: Processes images, detects tracks and obstacles
3. **SerialCommunicator**: Transmits control commands to the control system
4. **ConfigManager**: Manages JSON configuration parameters
5. **TrackVision**: Core state machine logic and processing

## 🔧 Software Dependencies

- OpenCV 4.x
- C++11 compatible compiler
- CMake 3.10+
- Optional: ZED SDK 3.x (required for ZED cameras)
- Optional: CUDA (required for ZED cameras)

## 🛠️ Building the Project

### Building on Ubuntu:

```bash
# Install necessary system packages
sudo apt update
sudo apt install build-essential cmake git libopencv-dev

# Clone the repository and build
git clone https://github.com/yourusername/blind-track-vision.git
cd blind-track-vision
mkdir build && cd build

# Build with standard camera support only
cmake ..
# Or build with ZED camera support
# cmake -DUSE_ZED_SDK=ON ..

make -j4
```

### Building on Windows:

1. Install OpenCV (prebuilt binaries recommended)
2. Install CMake and Visual Studio
3. Set OpenCV_DIR environment variable to point to your OpenCV build directory
4. If ZED camera support is needed, install ZED SDK and CUDA
5. Generate Visual Studio project files using CMake:
   ```
   cmake -B build
   # Or with ZED camera support
   # cmake -B build -DUSE_ZED_SDK=ON
   ```
6. Open the project and build

## 🚀 Running the Application

```bash
# Run from build directory
./blind_track_vision

# Use custom configuration file
./blind_track_vision -c /path/to/config.json
```

### 📡 Setting up Serial Communication

#### On Ubuntu:
1. Identify the serial device (usually `/dev/ttyUSB0` or `/dev/ttyACM0`):
   ```bash
   ls -l /dev/tty*
   ```

2. Set user permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod a+rw /dev/ttyUSB0
   ```

3. Update the `serial.port` setting in config/vision_config.json.

#### On Windows:
- Identify the COM port from Device Manager
- Update the `serial.port` setting in config/vision_config.json accordingly

## ⚙️ Configuration Parameters

The system uses a JSON configuration file (`config/vision_config.json`) with the following parameters:

### 📷 Camera Settings
- `camera.id` - Camera ID (default: 0)
- `camera.width` - Image width (default: 640)
- `camera.height` - Image height (default: 480)
- `camera.fps` - Target frame rate (default: 30)
- `camera.mode` - Camera mode ("standard" or "zed", default: "standard")
- `camera.depth_mode` - ZED camera depth mode ("PERFORMANCE", "QUALITY", or "ULTRA", default: "PERFORMANCE")

### 🔍 Track Detection Settings
- `detector.threshold` - Binarization threshold (default: 127)
- `detector.roi_y_start` - ROI start position as a fraction of image height (default: 0.6)
- `detector.roi_height` - ROI height as a fraction of image height (default: 0.3)
- `detector.roi_width` - ROI width as a fraction of image width (default: 0.5)
- `detector.canny_low` - Canny edge detection low threshold (default: 50)
- `detector.canny_high` - Canny edge detection high threshold (default: 150)
- `detector.hough_threshold` - Hough line detection threshold (default: 50)
- `detector.min_line_length` - Minimum line length (default: 30)
- `detector.max_line_gap` - Maximum line gap (default: 10)
- `detector.obstacle_contour_threshold` - Obstacle detection contour threshold (default: 5)

### 🚗 Obstacle Avoidance Settings
- `avoidance.preferred_direction` - Preferred avoidance direction ("right" or "left", default: "right")
- `avoidance.turn_time_ms` - Turn duration (milliseconds, default: 500)
- `avoidance.forward_time_ms` - Forward movement duration (milliseconds, default: 800)
- `avoidance.check_interval_ms` - Check interval (milliseconds, default: 100)
- `avoidance.min_line_detection_frames` - Consecutive frames needed to confirm track reacquisition (default: 3)
- `avoidance.obstacle_distance_threshold` - Obstacle detection distance threshold (meters, default: 1.0)
- `avoidance.clear_path_threshold` - Distance threshold to consider a path clear (meters, default: 1.5)

### 🔌 Serial Communication Settings
- `serial.port` - Serial port name (default on Windows: `COM3`, on Ubuntu: `/dev/ttyUSB0`)
- `serial.baudrate` - Serial baud rate (default: 115200)

### 🐞 Debug Settings
- `debug.show_view` - Enable visual debugging windows (default: true)

## 📊 Serial Communication Protocol

The system uses a binary structure-based communication protocol for efficient one-way data transmission from vision system to control system:

### Vision → Control System (VisionData Structure) (Active)

```cpp
#pragma pack(push, 1)
struct VisionData {
    uint8_t header = 0xCD;         // Frame header
    float line_offset = 0.0f;      // Track deviation (negative: left, positive: right)
    uint8_t line_found = 0;        // Track detection status: 0=not detected, 1=detected
    uint8_t obstacle_detected = 0; // Obstacle detection status: 0=none, 1=detected
    uint8_t avoid_direction = 0;   // Avoidance direction: 0=none, 1=left, 2=right
    float timestamp = 0.0f;        // Timestamp or frame number
    uint8_t tail = 0xDC;           // Frame tail
};
#pragma pack(pop)
```

### Control System → Vision (Stm32Data Structure) (Not Implemented)

```cpp
#pragma pack(push, 1)
struct Stm32Data {
    uint8_t header = 0xCD;       // Frame header
    uint8_t control_state = 0;   // Control state: 0=tracking, 1=avoiding, 2=avoid complete
    uint8_t motion_state = 0;    // Motion state: 0=stop, 1=forward, 2=turn left, 3=turn right
    float timestamp = 0.0f;      // Timestamp for synchronization
    uint8_t tail = 0xDC;         // Frame tail
};
#pragma pack(pop)
```

**Note:** The current implementation only uses one-way communication (Vision → Control). The receiving functionality (Control → Vision) is defined in the protocol but not actively used.

## ❓ Troubleshooting

### 📷 Camera Issues
- Ensure the camera is properly connected and drivers are correctly installed
- Check that the camera ID is correctly set
- Try reducing resolution or frame rate
- When using a ZED camera, ensure ZED SDK and CUDA are installed

### 🚧 Detection Issues
- Adjust `detector.threshold` to adapt to different lighting conditions
- Modify `detector.roi_*` parameters to capture the correct area in the image
- Adjust Canny and Hough parameters to optimize track detection

### ⚠️ Avoidance Issues
- Adjust `avoidance.turn_time_ms` and `avoidance.forward_time_ms` to match the vehicle's actual speed
- Ensure `detector.obstacle_contour_threshold` is appropriate for the current environment
- When using a ZED camera, adjust `avoidance.obstacle_distance_threshold` and `avoidance.clear_path_threshold`

## 👥 Contributing

Contributions are welcome! Feel free to submit Pull Requests or report issues.

## 📄 License

MIT License
