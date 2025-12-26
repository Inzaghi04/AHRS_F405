# F405_AHRS

**AHRS (Attitude and Heading Reference System)** running on **STM32F405** microcontroller, utilizing **MPU6000 IMU** and advanced filtering algorithms for accurate **Roll – Pitch – Yaw** estimation.

This project is designed for **flight controller / drone / UAV / autonomous robot** applications.

---

## 🚀 Key Features

- **MPU6000 IMU** data acquisition
  - Gyroscope
  - Accelerometer
- Noise filtering:
  - 2-pole Low-Pass Filter
- AHRS algorithms:
  - **Madgwick AHRS**
  - **Extended Kalman Filter (EKF)**
- Custom math library:
  - Vector2 / Vector3
  - Matrix3
  - Rotation utilities
- Modular design, easy to extend with PID / Flight Control

---

## 🧠 Algorithms Used

### 1️⃣ Madgwick AHRS

- Quaternion-based
- Fast convergence
- Suitable for real-time flight controller applications

**Directory:**
```
Core/Src/MadgwickAHRS/
├── MadgwickAHRS.c
└── MadgwickAHRS.h
```

---

### 2️⃣ Extended Kalman Filter (EKF)

- Advanced state filtering
- Reduces gyro drift
- Improves attitude stability

**Directory:**
```
Core/Src/EKF/
├── EKF.c
└── EKF.h
```

---

### 3️⃣ Low-Pass Filter (2-Pole)

- Reduces high-frequency noise from IMU
- Applied to accelerometer / gyroscope data

**Directory:**
```
Core/Src/LowPassFillter2p/
├── LowPassFilter.c
└── LowPassFilter.h
```

---

## 🧮 Math Library (Custom)

**Directory:**
```
Core/Src/Math/
├── ftype.h
├── vector2.c / .h
├── vector3.c / .h
├── matrix3.c / .h
└── rotations.h
```

**Features:**
- Vector & matrix operations
- Rotation conversions
- Used directly by EKF & AHRS

---

## 🔧 Hardware

| Component | Description |
|-----------|-------------|
| MCU | STM32F405 |
| IMU | MPU6000 |
| Clock | 168 MHz |
| IMU Interface | SPI |
| Debug | UART |

---

## 📁 Project Structure

```
F405_AHRS/
│
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f4xx_it.h
│   │   └── stm32f4xx_hal_conf.h
│   │
│   └── Src/
│       ├── main.c
│       ├── MPU6000/
│       │   ├── mpu6000.c
│       │   └── mpu6000.h
│       ├── MadgwickAHRS/
│       ├── EKF/
│       ├── LowPassFillter2p/
│       └── Math/
│
├── Drivers/
├── F405_AHRS.ioc
└── README.md
```

---

## 🔄 Main Processing Flow

1. Read MPU6000 data via SPI
2. Apply Low-Pass Filter for noise reduction
3. Calculate attitude:
   - Madgwick AHRS
   - EKF refinement
4. Output Roll / Pitch / Yaw
5. Ready for PID / Flight Control integration

---

## 📤 Output Example

```
ROLL  : 2.31 deg
PITCH : -1.12 deg
YAW   : 178.6 deg
```

---

## ⚠️ Current Limitations

**Inertial Navigation:**
- The current system uses **IMU-only** for position estimation
- **GPS** and **Barometer** are not yet integrated
- Inertial positioning accuracy degrades over time due to sensor drift
- Position estimates are **not suitable for long-term navigation** without correction

**Upcoming improvements:**
- GPS integration for absolute position correction
- Barometer for altitude fusion
- Sensor fusion (EKF with GPS + Baro + IMU) for accurate 3D positioning

---

## 🔧 Build & Flash

1. Open `F405_AHRS.ioc` with STM32CubeIDE
2. Generate code
3. Build project
4. Flash firmware using ST-Link
5. Debug via UART

---

## 🔮 Future Development

- Integrate PID controllers for Roll / Pitch / Yaw
- Connect Receiver (SBUS / PPM)
- PWM output for ESC control
- CAN / Telemetry support
- Magnetometer calibration & sensor alignment

---

## 📜 License

MIT License  
Free for education & research.