# ROS_HARDWARE

AnglersKing 自主导航小车底层驱动板固件，运行于 STM32F407IGT6。

## 硬件

| 项 | 参数 |
|---|------|
| **MCU** | STM32F407IGT6 (ARM Cortex-M4, 168MHz) |
| **RTOS** | FreeRTOS (CMSIS-OS v1) |
| **编译工具链** | arm-none-eabi-gcc (CMake + Ninja) |
| **调试** | CLion + OpenOCD / J-Link |

## 功能

- **电机控制**: 4 路 PWM 无刷电机驱动 (TIM3, CH1-CH4)
- **编码器读取**: 两路编码器速度反馈 (TIM1, TIM2)
- **IMU 姿态解算**: MPU6050 DMP 姿态融合 (I2C)
- **串口通信**: UART1 与上位机 ROS 通信，UART2 预留 (GPS)
- **PID 控制**: 直立环 PD + 速度环 PI + 转向环 PD
- **FreeRTOS 多任务**:
  - `Task_IMU` — 200Hz IMU 数据采集 + DMP 姿态解算
  - `Task_PID` — 100Hz PID 控制 + 电机输出
  - `Task_uart1` — 10Hz 数据上报 (里程计/IMU/电压)
  - `Task_uart2` — GPS 预留

## 软件架构

```
┌──────────────────────────────────────┐
│             FreeRTOS                 │
├─────────┬─────────┬────────┬────────┤
│ Task_IMU│Task_PID │TASK_U1 │TASK_U2 │
│  200Hz  │  100Hz  │  10Hz  │  (GPS) │
├─────────┴─────────┴────────┴────────┤
│  MPU6050  │ PID │ 编码器 │ BLDC PWM │
├──────────────────────────────────────┤
│       HAL (STM32F4xx)               │
├──────────────────────────────────────┤
│       Cortex-M4 (168MHz)            │
└──────────────────────────────────────┘
```

## 通信协议

与上位机 (ROS) 通过 UART1 串口通信，自定义二进制帧：

```
帧头(1B) + 速度数据 + IMU数据 + 电池电压 + 帧尾校验(1B)
```

| 字段 | 含义 | 方向 |
|------|------|------|
| `X_speed` | 底盘线速度 (m/s) | STM32 → ROS |
| `Z_speed` | 底盘角速度 (rad/s) | STM32 → ROS |
| `Link_Accelerometer` | 加速度计 XYZ 原始值 | STM32 → ROS |
| `Link_Gyroscope` | 陀螺仪 XYZ 原始值 | STM32 → ROS |
| `Source_Voltage` | 电池电压 (×100) | STM32 → ROS |

## 构建

### 依赖

- `arm-none-eabi-gcc` 交叉编译工具链
- CMake ≥ 3.24
- Ninja

```bash
# macOS 安装工具链
brew install --cask gcc-arm-embedded

# Ubuntu
sudo apt install gcc-arm-none-eabi cmake ninja-build
```

### 编译

```bash
mkdir build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug
ninja
```

产物：
- `clion_car_motor_hardware.elf` — 调试用
- `clion_car_motor_hardware.bin` — 烧录用
- `clion_car_motor_hardware.hex` — 备选烧录格式

### CLion 开发

直接用 CLion 打开项目根目录，CMakeLists.txt 已配置好交叉编译工具链。

## PID 控制参数

```c
// 直立环 (PD)
Balance_Kp = 200
Balance_Kd = 1

// 速度环 (PI)
Velocity_Kp = -52
Velocity_Ki = -0.26

// 转向环 (PD)
Turn_Kp = 18
Turn_Kd = 0.18
```

## 代码结构

```
Core/
├── Src/
│   ├── main.c                      # 入口 + 系统时钟配置
│   ├── freertos.c                  # FreeRTOS 任务创建
│   ├── car_task.c                  # 核心控制任务 (IMU采集/电机PID)
│   ├── comminicate.c               # 通信协议 + 运动学正解
│   ├── contrl.c                    # PID 控制器 + 编码器读取
│   ├── mpu6050.c                   # MPU6050 驱动
│   ├── bsp_bldc.c                  # BLDC 无刷电机底层驱动
│   ├── bsp_hall.c                  # 霍尔传感器驱动
│   └── retarget.c                  # printf 重定向到 UART1
├── Inc/                            # 头文件
├── Startup/                        # 启动文件
Drivers/
├── STM32F4xx_HAL_Driver/           # STM32 HAL 库
├── CMSIS/                          # Cortex-M4 CMSIS
└── MPU6050_eMPL/                   # MPU6050 DMP 库
Middlewares/
└── Third_Party/
    └── FreeRTOS/                   # FreeRTOS 内核 + CMSIS-OS 封装
```

## 原理说明

### 运动学正解 (Kinematics_Positive)

将上位机发送的线速度 vx、角速度 vz 解算为左右轮目标速度：

- 原地旋转 (vx=0): `Left = -Right = vz × 轮距/2`
- 直行 (vz=0): `Left = Right = vx`
- 曲线运动: `Left = vx - vz×轮距/2, Right = vx + vz×轮距/2`

### PID 控制回路

```
1. 直立环 PD: Balance_PWM = Kp×倾斜角偏差 + Kd×角速度
2. 速度环 PI: Velocity_PWM = Kp×速度偏差 + Ki×速度积分
3. 转向环 PD: Turn_PWM = Kp×转向偏差 + Kd×Z轴角速度
4. 最终输出:
   Motor_L = Balance + Velocity + Turn
   Motor_R = Balance + Velocity - Turn
```

### 通信帧格式

自定义二进制协议，每个数据包 `struct` 直接序列化为字节流发送，无需 JSON/ASCII 解析，适合 MCU 低延迟场景。

## 关联项目

- [anglersking_auto_car](https://github.com/anglersking/anglersking_auto_car) — ROS 上位机端
- 本仓库为同一小车的 STM32 底层驱动板固件
