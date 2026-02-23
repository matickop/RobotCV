# RobotCV - Vision-Guided Robot Pick and Place System

Computer vision system for Universal Robots enabling automated object detection, localization, and pick-and-place operations using real-time image processing and RTDE control interface.

**Key Features:**
- Real-time vision processing with industrial cameras (Basler/Pypylon)
- RTDE-based robot control for UR16e
- Socket-based Robotiq gripper integration
- Motion monitoring and safety features
- Adaptive grasping with vision feedback

---

## Project Overview

This system combines computer vision and robotic control to automate pick-and-place tasks:

1. **Vision Module** (`kamera_module.py`) - Camera calibration, object detection, pose estimation
2. **Robot Control** (`robot_module.py`) - RTDE interface, inverse kinematics, motion planning
3. **Gripper Control** (`robotiq_gripper.py`) - Socket-based communication with Robotiq Hand-E
4. **GUI** (`GUI.py`) - User interface for operation and configuration
5. **Main program** (`main.py`) - Program for operating the robot

---

## Hardware Requirements

- **Robot**: Universal Robots UR16e
- **Gripper**: Robotiq Gripper 2F-140
- **Camera**: Basler industrial camera via GigE
- **Network**: Ethernet connection to robot controller and Basler camera (static IP recommended)
- **Computer**: Linux (Fedora/Ubuntu) or Windows with Python 3.11+

---

## Installation

### Prerequisites

- Python 3.11 or higher
- Linux (tested on Fedora 41) or Windows 10/11
- Connection with robot and camera via TCP/IP
- Camera drivers (Pypylon for Basler cameras)

### Setup

#### Linux (Fedora/Ubuntu)

```bash
# Clone repository
git clone git@github.com:matickop/RobotCV.git
cd RobotCV

# Create virtual environment
python3.11 -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
python -c "import cv2, numpy, rtde_control; print('Dependencies OK')"
```

#### Windows

```powershell
# Clone repository
git clone git@github.com:matickop/RobotCV.git
cd RobotCV

# Create virtual environment
python -m venv .venv
.venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
python -c "import cv2, numpy, rtde_control; print('Dependencies OK')"
```

---

## Quick Start

### 1. Network Configuration

Configure robot IP address in `main.py`:

```python
ROBOT_IP = "192.168.1.100"  # Replace with your robot's IP
```

Verify RTDE interface is enabled:
- Robot Teach Pendant → Settings → System → RTDE
- Enable Real-Time Data Exchange


### 2. Run Application

```bash
# GUI mode (recommended)
python GUI.py

# Headless mode
python main.py
```

### 3. Basic Pick-and-Place Workflow

1. **Initialize System**: Launch GUI  
2. **Start main loop**: Click `Celoten loop`
