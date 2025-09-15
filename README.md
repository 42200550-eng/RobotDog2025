# 🤖 Quadruped Robot - ESP32 Implementation

## 📋 Mô tả dự án

Đây là dự án robot chó bốn chân được lập trình trên **ESP32** sử dụng **Arduino Framework** và **PlatformIO**. Robot có khả năng di chuyển theo nhiều kiểu bước khác nhau và được điều khiển từ xa qua tay cầm PlayStation 4.

## ✨ Tính năng chính

- 🚶 **Nhiều chế độ di chuyển**: Trot, Yaw, Pitch-Roll, Object Detection
- 🎮 **Điều khiển PS4**: Kết nối Bluetooth với tay cầm PS4
- 🧮 **Inverse Kinematics**: Tính toán động học ngược thời gian thực
- 🔧 **Servo Calibration**: Hiệu chỉnh servo qua Serial interface
- 📡 **Wireless Control**: Điều khiển không dây hoàn toàn

## 🔧 Phần cứng yêu cầu

### Vi điều khiển
- **ESP32** (Feather ESP32 hoặc tương tự)

### Actuators
- **12x Servo Motors** (3 servo cho mỗi chân)
  - Shoulder joint (khớp vai)
  - Upper joint (khớp đùi) 
  - Lower joint (khớp cẳng chân)

### Drivers & Controllers
- **Adafruit PWM Servo Driver** (PCA9685)
- **PlayStation 4 Controller**

### Cấu trúc chân robot
```
     [FRONT]           [BACK]
LEFT  │    │  RIGHT    LEFT  │    │  RIGHT
     [1]  [2]          [0]  [3]

Mỗi chân có 3 khớp:
- Joint 0: Lower (cẳng chân)
- Joint 1: Upper (đùi)  
- Joint 2: Shoulder (vai)
```

## 📁 Cấu trúc code

```
📁 src/
├── 📄 source_esp32.cpp    # File chính, điều khiển tổng thể
├── 📄 kinematics.cpp      # Tính toán động học robot
├── 📄 hardware.cpp        # Điều khiển phần cứng, servo
└── 📄 datatypes.h         # Định nghĩa kiểu dữ liệu

📁 include/               # Header files
📁 lib/                   # Local libraries  
📁 test/                  # Test files
📄 platformio.ini         # PlatformIO configuration
```

## ⚙️ Thông số kỹ thuật

### Kích thước robot
- **Body size**: 300×40×180 mm
- **Bone length**: 105 mm
- **Step extent**: 40×40×26 mm

### Tần số hoạt động
- **Loop frequency**: 440 Hz
- **Servo PWM frequency**: 60 Hz
- **Serial baud rate**: 115200

### Chế độ hoạt động (States)
- **0**: Idle (nghỉ)
- **1**: Trot (bước rảo)
- **2**: Yaw (xoay)
- **3**: Pitch-roll (nghiêng)
- **4**: Object detection (phát hiện vật thể)

## 🎮 Hướng dẫn điều khiển

### PlayStation 4 Controller

#### Analog Sticks
- **Left Stick**:
  - ↕️ **Y-axis**: Di chuyển tiến/lùi
  - ↔️ **X-axis**: Di chuyển trái/phải
- **Right Stick**:
  - ↕️ **Y-axis**: Điều chỉnh độ cao chân
  - ↔️ **X-axis**: Xoay robot (yaw)

#### Buttons
- **Touchpad**: Chuyển đổi giữa các chế độ hoạt động

### Deadzone Settings
- **Stick minimum**: 6.0 (vùng chết của analog stick)
- **Precision**: 0.001 mm

## 🚀 Cài đặt và sử dụng

### 1. Cài đặt môi trường

#### Option A: PlatformIO CLI
```bash
pip install platformio
```

#### Option B: VS Code Extension
- Cài đặt **PlatformIO IDE** extension trong VS Code

### 2. Clone và build project
```bash
git clone <repository-url>
cd robotdog
pio run
```

### 3. Upload code lên ESP32
```bash
pio run --target upload
```

### 4. Monitor Serial Output
```bash
pio device monitor
```

## 🔧 Cấu hình

### 1. PS4 Controller MAC Address
Thay đổi MAC address trong `source_esp32.cpp`:
```cpp
PS4.begin("F8:C3:9E:3F:F8:10"); // Thay bằng MAC của controller
```

### 2. Servo Calibration
Sử dụng Serial interface để hiệu chỉnh servo:
```
Format: <leg> <joint> <servo_value>
Example: 0 1 90
```

### 3. Hardware Initialization
Kiểm tra kết nối I2C và PWM driver trong `hardware.cpp`

## 📐 Tính toán Kinematics

### Inverse Kinematics Algorithm
Robot sử dụng thuật toán động học ngược để:
1. Chuyển đổi vị trí mong muốn của chân → góc servo
2. Tính toán góc khớp dựa trên geometry tam giác
3. Áp dụng các gait patterns khác nhau

### Gait Functions
- **Trot Gait**: Bước rảo tự nhiên (2 chân cùng lúc)
- **Yaw Axis**: Xoay quanh trục dọc
- **Pitch-Roll**: Nghiêng trước/sau, trái/phải

## 🔌 Kết nối phần cứng

### PWM Servo Driver (PCA9685)
```
ESP32 -> PCA9685
SDA   -> SDA
SCL   -> SCL  
VCC   -> 3.3V
GND   -> GND
```

### Servo Mapping
```cpp
// Leg 0 (Back Left)
servo00 -> Lower joint
servo02 -> Upper joint  
servo04 -> Shoulder joint

// Leg 1 (Front Left)
servo15 -> Lower joint
servo13 -> Upper joint
servo05 -> Shoulder joint

// Leg 2 (Front Right) 
servo14 -> Lower joint
servo12 -> Upper joint
servo07 -> Shoulder joint

// Leg 3 (Back Right)
servo01 -> Lower joint
servo03 -> Upper joint
servo06 -> Shoulder joint
```

## 🐛 Troubleshooting

### Common Issues

1. **PS4 Controller không kết nối**
   - Kiểm tra MAC address
   - Đảm bảo controller ở pairing mode
   - Reset Bluetooth trên ESP32

2. **Servo không hoạt động**
   - Kiểm tra nguồn cấp servo (5V, đủ ampe)
   - Verify I2C connection
   - Test PWM signals

3. **Robot di chuyển không ổn định**
   - Calibrate servos
   - Kiểm tra `base_offset` values
   - Adjust kinematics parameters

### Serial Debug Commands

```bash
# Mode 0: Servo calibration
<leg> <joint> <servo_value>

# Mode 1: Manual control  
<direction_x> <direction_y> <turn>
```

## 📚 Dependencies

### Libraries được sử dụng
- **Adafruit PWM Servo Driver Library**: Điều khiển servo qua I2C
- **PS4 Controller Host**: Kết nối với PS4 controller

### PlatformIO Configuration
```ini
[env:featheresp32]
platform = espressif32
board = featheresp32
framework = arduino
monitor_speed = 115200
lib_deps = 
    pablomarquez76/PS4_Controller_Host@^1.1.1
    adafruit/Adafruit PWM Servo Driver Library
```

## 🤝 Contributing

1. Fork the project
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under CC-BY License - see the original [repository](https://github.com/maestrakos/warp) for details.

## 👨‍💻 Authors

- **Original Author**: Alexandros Petkos (maestrakos)
- **Date**: 3/10/2020
- **Updates**: Available at https://github.com/maestrakos/warp

## 🙏 Acknowledgments

- Thanks to the original warp_kinematics project
- Adafruit for excellent hardware libraries
- ESP32 community for support and examples

---

### 📞 Support

Nếu gặp vấn đề, vui lòng:
1. Kiểm tra [Issues](../../issues) 
2. Đọc documentation
3. Post question với detailed error logs

**Happy Robotics! 🤖✨**
