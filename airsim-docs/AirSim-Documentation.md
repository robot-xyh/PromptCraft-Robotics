# AirSim Documentation

> **Notice**: This repository will be archived. Microsoft is releasing a new platform - **Microsoft Project AirSim** - for aerospace simulation. Visit https://aka.ms/projectairsim for more information.

---

# Table of Contents

1. [Home](#home)
2. [Building AirSim](#building-airsim)
   - [Build on Windows](#build-on-windows)
3. [Using AirSim](#using-airsim)
   - [Core APIs](#core-apis)
   - [Image APIs](#image-apis)
   - [Settings](#settings)
   - [Car Mode](#car-mode)
   - [Multiple Vehicles](#multiple-vehicles)
4. [Sensors](#sensors)
   - [Sensors Overview](#sensors-overview)
   - [LIDAR](#lidar)
5. [Design](#design)
   - [Simple Flight](#simple-flight)
6. [External Flight Controllers](#external-flight-controllers)
   - [PX4 Setup](#px4-setup)
7. [Tutorials](#tutorials)
   - [Reinforcement Learning](#reinforcement-learning)
8. [Support](#support)
   - [FAQ](#faq)

---

# Home

AirSim is a simulator for drones, cars and more, built on Unreal Engine (with experimental Unity support). It is open-source, cross platform, and supports software-in-the-loop simulation with popular flight controllers such as PX4 & ArduPilot.

## Goals

- Platform for AI research: deep learning, computer vision, reinforcement learning
- Exposes APIs to retrieve data and control vehicles in a platform independent way

## How to Get It

| Platform | Options |
|----------|---------|
| Windows | Download binaries / Build from source |
| Linux | Download binaries / Build from source |
| macOS | Build from source |

## Usage Modes

- **Manual Drive**: RC for drones, arrow keys for cars
- **Programmatic Control**: APIs via RPC (Python, C++, C#, Java)
- **Computer Vision Mode**: No vehicle/physics, keyboard navigation
- **Weather Effects**: Press F10 for options

---

# Building AirSim

## Build on Windows

### Install Unreal Engine

1. Download Epic Games Launcher
2. Install Unreal Engine >= 4.27

### Build AirSim

1. Install Visual Studio 2022 with:
   - Desktop Development with C++
   - Windows 10 SDK 10.0.19041
   - Latest .NET Framework SDK

2. Start Developer Command Prompt for VS 2022

3. Clone and build:
```bash
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
build.cmd
```

### Build Unreal Project

1. Double click on `.sln` file in `Unreal\Environments\Blocks`
2. Select project as Start Up project
3. Set Build config to "Develop Editor" and x64
4. Press Play in Unreal Editor

**Tip**: Go to Edit->Editor Preferences, search "CPU", uncheck "Use Less CPU when in Background"

---

# Using AirSim

## Core APIs

### Python Quickstart

```bash
pip install msgpack-rpc-python
pip install airsim
```

### Hello Car

```python
import airsim
import time

client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

while True:
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    car_controls.throttle = 1
    car_controls.steering = 1
    client.setCarControls(car_controls)
    time.sleep(1)

    responses = client.simGetImages([
        airsim.ImageRequest(0, airsim.ImageType.DepthVis),
        airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)])
```

### Hello Drone

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()

responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),
    airsim.ImageRequest("1", airsim.ImageType.DepthPlanar, True)])
```

### Common APIs

| API | Description |
|-----|-------------|
| `reset()` | Reset vehicle to original state |
| `confirmConnection()` | Check connection status |
| `enableApiControl(is_enabled)` | Enable/disable API control |
| `isApiControlEnabled()` | Check if API control is enabled |
| `ping()` | Test connection |
| `simPrintLogMessage(message)` | Print message in simulator |
| `simGetObjectPose(name)` | Get object pose |
| `simSetObjectPose(name, pose)` | Set object pose |

### Pause and Continue APIs

```python
client.pause(True)   # Pause simulation
client.pause(False)  # Continue simulation
client.continueForTime(seconds)  # Run for specified time then pause
```

### Collision API

```python
collision_info = client.simGetCollisionInfo()
```

### Weather APIs

```python
client.simEnableWeather(True)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.25)
```

**WeatherParameter values**: Rain, Roadwetness, Snow, RoadSnow, MapleLeaf, RoadLeaf, Dust, Fog

### Wind API

```python
wind = airsim.Vector3r(20, 0, 0)  # 20m/s North
client.simSetWind(wind)
```

### Recording APIs

```python
client.startRecording()
client.stopRecording()
client.isRecording()
```

### Coordinate System

All AirSim API uses NED coordinate system:
- **+X**: North
- **+Y**: East
- **+Z**: Down

All units are SI (meters, seconds).

### Multirotor APIs

| API | Description |
|-----|-------------|
| `getMultirotorState()` | Get complete state |
| `takeoffAsync()` | Take off |
| `landAsync()` | Land |
| `moveToPositionAsync(x, y, z, velocity)` | Move to position |
| `moveByVelocityAsync(vx, vy, vz, duration)` | Move by velocity |
| `moveOnPathAsync(path, velocity)` | Follow path |
| `rotateToYawAsync(yaw)` | Rotate to yaw |
| `hoverAsync()` | Hover |
| `goHomeAsync()` | Return to launch |

### Car APIs

| API | Description |
|-----|-------------|
| `setCarControls(controls)` | Set throttle, steering, brake |
| `getCarState()` | Get speed, gear, kinematics |
| `getCarControls()` | Get current controls |

---

## Image APIs

### Getting a Single Image

```python
import airsim

client = airsim.MultirotorClient()
png_image = client.simGetImage("0", airsim.ImageType.Scene)
```

### Getting Multiple Images

```python
responses = client.simGetImages([
    airsim.ImageRequest(0, airsim.ImageType.Scene),           # PNG
    airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),  # Uncompressed
    airsim.ImageRequest(1, airsim.ImageType.DepthPlanar, True)     # Float depth
])
```

### Using with NumPy

```python
import numpy as np

responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
])
response = responses[0]

img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
img_rgb = img1d.reshape(response.height, response.width, 3)
img_rgb = np.flipud(img_rgb)
```

### Available Cameras

**Car**: front_center, front_right, front_left, fpv, back_center

**Multirotor**: front_center, front_right, front_left, bottom_center, back_center

**Backward compatible IDs**: "0", "1", "2", "3", "4"

### ImageType Values

| Type | Value | Description |
|------|-------|-------------|
| Scene | 0 | RGB |
| DepthPlanar | 1 | Depth (planar) |
| DepthPerspective | 2 | Depth (perspective) |
| DepthVis | 3 | Depth visualization |
| DisparityNormalized | 4 | Disparity |
| Segmentation | 5 | Segmentation |
| SurfaceNormals | 6 | Surface normals |
| Infrared | 7 | Infrared |
| OpticalFlow | 8 | Optical flow |
| OpticalFlowVis | 9 | Optical flow vis |

### Camera APIs

```python
# Get camera info
info = client.simGetCameraInfo("front_center")

# Set camera pose
camera_pose = airsim.Pose(
    airsim.Vector3r(0, 0, 0),
    airsim.to_quaternion(0.261799, 0, 0)  # pitch, roll, yaw in radians
)
client.simSetCameraPose("0", camera_pose)

# Set FOV
client.simSetCameraFov("0", 90)
```

### Segmentation

```python
# Set object ID
client.simSetSegmentationObjectID("Ground", 20)

# Use regex for multiple objects
client.simSetSegmentationObjectID("ground[\\w]*", 21, True)
```

---

## Settings

Settings are stored in `~/Documents/AirSim/settings.json`

### SimMode

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Car"
}
```

Values: "", "Multirotor", "Car", "ComputerVision"

### ViewMode

Values: FlyWithMe, GroundObserver, Fpv, Manual, SpringArmChase, NoDisplay

### Complete Settings Example

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockSpeed": 1,
    "LocalHostIp": "127.0.0.1",
    "ApiServerPort": 41451,
    "Wind": { "X": 0, "Y": 0, "Z": 0 },
    "CameraDefaults": {
        "CaptureSettings": [{
            "ImageType": 0,
            "Width": 256,
            "Height": 144,
            "FOV_Degrees": 90
        }]
    },
    "OriginGeopoint": {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 122
    },
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": 0, "Y": 0, "Z": 0
        }
    }
}
```

### Recording Settings

```json
"Recording": {
    "RecordOnMove": false,
    "RecordInterval": 0.05,
    "Enabled": false,
    "Cameras": [{
        "CameraName": "0",
        "ImageType": 0,
        "PixelsAsFloat": false,
        "Compress": true
    }]
}
```

### ClockSpeed

- `5.0`: Simulation 5x faster than real time
- `0.1`: Simulation 10x slower than real time
- `1.0`: Real time

---

## Car Mode

### Settings

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Car"
}
```

### Manual Driving

- Arrow keys to drive
- Spacebar for handbrake
- F key for FPV view
- / key for chase view
- F1 for more shortcuts

### Cameras

Default cameras: center, left, right, driver, reverse

---

## Multiple Vehicles

### Creating Multiple Cars

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Car",
    "Vehicles": {
        "Car1": {
            "VehicleType": "PhysXCar",
            "X": 4, "Y": 0, "Z": -2
        },
        "Car2": {
            "VehicleType": "PhysXCar",
            "X": -4, "Y": 0, "Z": -2,
            "Yaw": 90
        }
    }
}
```

### Creating Multiple Drones

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "X": 4, "Y": 0, "Z": -2
        },
        "Drone2": {
            "VehicleType": "SimpleFlight",
            "X": 8, "Y": 0, "Z": -2
        }
    }
}
```

### Using APIs

```python
client.listVehicles()  # Returns ['Car1', 'Car2']

# Control specific vehicle
client.moveToPositionAsync(10, 0, -10, 5, vehicle_name="Drone1")
```

### Creating Vehicles at Runtime

```python
success = client.simAddVehicle(
    vehicle_name="NewDrone",
    vehicle_type="simpleflight",
    pose=airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.Quaternionr()),
    pawn_path=""
)
```

---

# Sensors

## Sensors Overview

| Sensor | Type ID |
|--------|---------|
| Camera | N/A |
| Barometer | 1 |
| IMU | 2 |
| GPS | 3 |
| Magnetometer | 4 |
| Distance Sensor | 5 |
| Lidar | 6 |

### Default Sensors by SimMode

**Multirotor**: IMU, Magnetometer, GPS, Barometer

**Car**: GPS

**ComputerVision**: None

### Configuring Sensors

```json
"DefaultSensors": {
    "Barometer": {
        "SensorType": 1,
        "Enabled": true
    },
    "Imu": {
        "SensorType": 2,
        "Enabled": true
    },
    "Gps": {
        "SensorType": 3,
        "Enabled": true
    },
    "Magnetometer": {
        "SensorType": 4,
        "Enabled": true
    }
}
```

### Sensor APIs

```python
barometer_data = client.getBarometerData()
imu_data = client.getImuData()
gps_data = client.getGpsData()
magnetometer_data = client.getMagnetometerData()
distance_data = client.getDistanceSensorData()
lidar_data = client.getLidarData()
```

---

## LIDAR

### Configuration

```json
"Sensors": {
    "LidarSensor1": {
        "SensorType": 6,
        "Enabled": true,
        "NumberOfChannels": 16,
        "RotationsPerSecond": 10,
        "PointsPerSecond": 100000,
        "X": 0, "Y": 0, "Z": -1,
        "Roll": 0, "Pitch": 0, "Yaw": 0,
        "VerticalFOVUpper": -15,
        "VerticalFOVLower": -25,
        "HorizontalFOVStart": -20,
        "HorizontalFOVEnd": 20,
        "DrawDebugPoints": true,
        "DataFrame": "SensorLocalFrame"
    }
}
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| NumberOfChannels | Number of lasers |
| Range | Range in meters |
| PointsPerSecond | Points captured per second |
| RotationsPerSecond | Rotations per second |
| HorizontalFOVStart/End | Horizontal FOV (degrees) |
| VerticalFOVUpper/Lower | Vertical FOV (degrees) |
| X, Y, Z | Position relative to vehicle (NED, meters) |
| Roll, Pitch, Yaw | Orientation (degrees) |
| DataFrame | "VehicleInertialFrame" or "SensorLocalFrame" |

### API Usage

```python
lidar_data = client.getLidarData()
points = lidar_data.point_cloud  # Flat array [x,y,z, x,y,z, ...]
pose = lidar_data.pose
```

---

# Design

## Simple Flight

AirSim's built-in flight controller. Used by default - no setup required.

### Advantages

- Zero setup
- Steppable clock (can pause simulation)
- Cross-platform, header-only C++

### Control Modes

- Angle rate
- Angle level
- Velocity
- Position

Uses cascade of PID controllers internally.

### Configuration

```json
"Vehicles": {
    "SimpleFlight": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Armed"
    }
}
```

Set to "Inactive" to start disarmed:

```json
"DefaultVehicleState": "Inactive"
```

### API Control Settings

```json
"Vehicles": {
    "SimpleFlight": {
        "VehicleType": "SimpleFlight",
        "AllowAPIAlways": true,
        "RC": {
            "RemoteControlID": 0,
            "AllowAPIWhenDisconnected": true
        }
    }
}
```

---

# External Flight Controllers

## PX4 Setup

### Supported Hardware

- Pixhawk PX4 2.4.8
- PixFalcon
- PixRacer
- Pixhawk 2.1
- Pixhawk 4 mini (Holybro)
- Pixhawk 4 (Holybro)

### Hardware-in-Loop Setup

1. Bind RC receiver with transmitter
2. Download QGroundControl
3. Flash latest PX4 firmware
4. Configure for HIL simulation (select "HIL Quadrocopter X")
5. Calibrate Radio in QGC
6. Configure Flight Modes

### Settings for PX4

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": true,
            "LockStep": true,
            "Sensors": {
                "Barometer": {
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1
            }
        }
    }
}
```

### FAQ

**Drone goes crazy**: Check drone doesn't fall large distance at start. Use QGC to verify arm/takeoff works.

**Not finding Pixhawk**: Check SerialPort setting. Windows: use Device Manager to find COM port. Linux: use `/dev/serial/by-id/...`

**Takeoff denied**: Wait for GPS lock (home position set).

**Drone lands after reaching target**: Set `COM_OBL_ACT` to 1 for hover instead of land.

---

# Tutorials

## Reinforcement Learning

Uses OpenAI gym wrapper with stable-baselines3.

### Installation

```bash
pip install stable-baselines3
```

### Car RL Example

```python
from stable_baselines3 import DQN

model = DQN(
    "CnnPolicy",
    env,
    learning_rate=0.00025,
    batch_size=32,
    buffer_size=500000,
    exploration_fraction=0.1,
    device="cuda",
    tensorboard_log="./tb_logs/",
)

model.learn(total_timesteps=100000)
```

### Actions (Car)

| Action | Description |
|--------|-------------|
| 0 | Brake |
| 1 | Straight + throttle |
| 2 | Full-left + throttle |
| 3 | Full-right + throttle |
| 4 | Half-left + throttle |
| 5 | Half-right + throttle |

### Actions (Drone)

| Action | Direction |
|--------|-----------|
| 0 | +X (forward) |
| 1 | +Y (right) |
| 2 | +Z (down) |
| 3 | -X (backward) |
| 4 | -Y (left) |
| 5 | -Z (up) |
| 6 | Hover |

### Reward Function Example

```python
def compute_reward(car_state):
    # Combination of speed and distance from center line
    reward_dist = math.exp(-beta * dist) - 0.5
    reward_speed = (car_state.speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED) - 0.5
    return reward_dist + reward_speed
```

---

# Support

## FAQ

### General Issues

**Unreal editor slow in background**: Edit > Editor Preferences > uncheck "Use Less CPU when in Background"

**Mouse disappears**: Alt+TAB to switch windows. Or disable mouse capture in Project Settings > Input.

**Settings file location**: `~/Documents/AirSim/settings.json`

**How to arm drone**: SimpleFlight is already armed. PX4: hold both sticks down and inward.

**API TypeError with AsyncIOLoop**:
```bash
pip install --upgrade msgpack-rpc-python
```

**Eigen not found**: Build AirSim first (run build.cmd)

### Hardware Requirements

| Environment | Requirements |
|-------------|--------------|
| Blocks | Typical laptop |
| Neighborhood Pack | GPU with 4GB+ RAM |
| Open World | GPU with 8GB+ RAM |

Recommended: 32GB RAM, NVIDIA TitanX, fast SSD

---

# Quick Reference

## Python Client Setup

```python
import airsim

# Drone
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Car
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
```

## Common Drone Operations

```python
# Takeoff
client.takeoffAsync().join()

# Move to position (NED: z negative = up)
client.moveToPositionAsync(10, 0, -10, 5).join()

# Get position
state = client.getMultirotorState()
pos = state.kinematics_estimated.position
print(f"x={pos.x_val}, y={pos.y_val}, z={pos.z_val}")

# Land
client.landAsync().join()

# Disarm
client.armDisarm(False)
client.enableApiControl(False)
```

## Common Car Operations

```python
car_controls = airsim.CarControls()
car_controls.throttle = 0.5
car_controls.steering = 0.0
car_controls.brake = 0.0
client.setCarControls(car_controls)

state = client.getCarState()
print(f"Speed: {state.speed}")
```

## Image Capture

```python
# Single image
png = client.simGetImage("front_center", airsim.ImageType.Scene)

# Multiple images
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene),
    airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True)
])
```

---

*Documentation source: https://microsoft.github.io/AirSim/*

*Copyright 2021 Microsoft Research - MIT License*
