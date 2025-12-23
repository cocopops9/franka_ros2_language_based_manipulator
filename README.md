# franka_ros2_language_based_manipulator

CIR-MAI course project: ROS2 modular system for tool manipulation using voice commands and vision inputs.

## Overview

This project integrates three main components to enable language-based manipulation of a Franka Emika robot:

1. **HMI Package** - Voice control interface using OpenAI Whisper for speech recognition
2. **Vision Module** - YOLO-based object detection with ArUco marker tracking
3. **Monitor/Controller** - MoveIt-based robot control for the Franka FR3

## Prerequisites

### System Requirements
- **Ubuntu 22.04** (recommended for ROS 2 Humble)
- **ROS 2 Humble** installed and sourced
- **Python 3.10+**
- **Docker** 20.10+ (for HMI package)
- **OpenAI API Key** (for voice recognition)

### ROS 2 Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-franka-description \
  ros-humble-franka-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs
```

### Franka Packages
If you don't have the Franka ROS 2 packages installed:
```bash
cd ~/ros2_ws/src
git clone https://github.com/frankaemika/franka_ros2.git
cd ~/ros2_ws
colcon build
```

## Installation

### 1. Clone the Repository
```bash
cd ~/ros2_ws/src
git clone <repository-url> franka_ros2_language_based_manipulator
cd ~/ros2_ws
```

### 2. Install Python Dependencies

**Vision Module:**
```bash
cd ~/ros2_ws/src/franka_ros2_language_based_manipulator/vision_module
pip install -r requirements.txt
```

**HMI Package:**
```bash
cd ~/ros2_ws/src/franka_ros2_language_based_manipulator/hmi_pkg
pip install -r requirements.txt
```

### 3. Build the ROS 2 Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select vision_module monitor_cpp
source install/setup.bash
```

## Running the System

### Option A: Run All Components Together

**Terminal 1: Start ROS 2 Core Components**
```bash
cd ~/ros2_ws
source install/setup.bash

# Launch the Franka robot (if using real hardware)
ros2 launch franka_bringup franka.launch.py robot_ip:=<your-robot-ip>

# Or use simulation/RViz for testing
ros2 launch franka_moveit_config demo.launch.py
```

**Terminal 2: Start Vision Module**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run vision_module vision_node

# Or use the launch file
ros2 launch vision_module vision_launch.py
```

**Terminal 3: Start Robot Controller**
```bash
cd ~/ros2_ws
source install/setup.bash

# Example: Move robot to a specific pose
ros2 launch monitor_cpp move_robot.launch.py x:=0.5 y:=0.0 z:=0.4 r:=180.0 p:=0.0 yw:=45.0
```

**Terminal 4: Start HMI Voice Control**
```bash
cd ~/ros2_ws/src/franka_ros2_language_based_manipulator/hmi_pkg

# Set your OpenAI API key
export OPENAI_API_KEY='sk-your-api-key-here'

# Enable ROS 2 integration
export HMI_ROS_ENABLE=1

# Start with Docker (recommended)
docker-compose up -d

# Or run directly with Python
python nodes/hmi_server.py
```

**Access the Web Interface:**
Open your browser to `http://localhost:8000`

### Option B: Run Components Individually

#### Vision Module Only
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run vision_module vision_node
```

Topics published:
- `/vision/detected_objects` - Detected objects with poses
- `/vision/markers` - ArUco marker information

#### Robot Controller Only
```bash
cd ~/ros2_ws
source install/setup.bash

# Move to a specific pose
ros2 launch monitor_cpp move_robot.launch.py \
  x:=0.5 y:=0.2 z:=0.3 \
  r:=180.0 p:=0.0 yw:=45.0
```

#### HMI Voice Control Only
See the detailed [HMI Package README](hmi_pkg/hmi_pkg_readme.md) for comprehensive instructions.

Quick start:
```bash
cd hmi_pkg
export OPENAI_API_KEY='sk-your-key'
docker-compose up -d
```

## System Architecture

```
┌─────────────────┐
│   Web Browser   │  Voice commands via microphone
│   (localhost:8000) │
└────────┬────────┘
         │ WebSocket
┌────────▼────────┐
│   HMI Package   │  Speech recognition (Whisper)
│                 │  Natural Language Understanding
└────────┬────────┘
         │ ROS 2 Topics
         │ /hmi/intent_raw
┌────────▼────────────────────────┐
│      ROS 2 Middleware           │
└──┬─────────────────────────┬───┘
   │                         │
   │ /vision/detected_objects│
┌──▼───────────┐      ┌─────▼─────────┐
│ Vision Module│      │   Controller   │
│   (YOLO +    │      │   (MoveIt)     │
│   ArUco)     │      │                │
└──────────────┘      └────────┬───────┘
                              │
                       ┌──────▼───────┐
                       │ Franka Robot │
                       └──────────────┘
```

## Voice Commands

Once the HMI is running, you can use these commands:

**Actions:**
- "pick [object]" - Pick up an object
- "release" - Release the held object
- "next" - Move to the next step
- "stop" - Stop current action
- "cancel" - Cancel operation

**Objects:**
- cup, bottle, mug, can

**Examples:**
- "pick cup"
- "release bottle"
- "next"

## Configuration

### Vision Module
Edit `vision_module/config/yolo.yaml` for YOLO model settings:
```yaml
model_path: "yolov8n.pt"
confidence_threshold: 0.5
```

Edit `vision_module/config/tools.json` for tool/object definitions.

### HMI Package
Edit `hmi_pkg/config/capabilities.json` to customize available actions and objects.

Environment variables for HMI (set in `hmi_pkg/docker-compose.yml`):
- `OPENAI_API_KEY` - Your OpenAI API key (required)
- `HMI_ROS_ENABLE` - Set to `1` to enable ROS 2 integration
- `WHISPER_MODEL` - Whisper model to use (default: `whisper-1`)
- `WHISPER_LANG` - Language code (default: `en`)

### Robot Controller
The `monitor_cpp` package accepts pose parameters via launch arguments:
- `x`, `y`, `z` - Target position (meters)
- `r`, `p`, `yw` - Roll, pitch, yaw (degrees)

## Monitoring Topics

Check what's being published:
```bash
# List all topics
ros2 topic list

# Monitor vision detections
ros2 topic echo /vision/detected_objects

# Monitor HMI intents
ros2 topic echo /hmi/intent_raw

# Monitor HMI transcriptions
ros2 topic echo /hmi/asr_text
```

## Troubleshooting

### Vision Module Issues
**No detections appearing:**
```bash
# Check if the node is running
ros2 node list | grep vision

# Verify camera/image source is publishing
ros2 topic list | grep image
ros2 topic hz /camera/image_raw

# Check YOLO model is loaded
ros2 topic echo /rosout | grep vision
```

### Robot Controller Issues
**MoveIt planning fails:**
```bash
# Ensure MoveIt is running
ros2 node list | grep move_group

# Check robot description is loaded
ros2 param list /move_group | grep robot_description

# Verify joint states are published
ros2 topic hz /joint_states
```

### HMI Issues
**No transcription:**
- Verify OpenAI API key is set correctly
- Check Docker logs: `docker-compose logs -f`
- Test microphone access in browser
- Increase sensitivity: `WHISPER_MIN_RMS=0.001`

**ROS topics not publishing:**
- Ensure `HMI_ROS_ENABLE=1` in `docker-compose.yml`
- Check ROS 2 is accessible from Docker container
- Verify topics: `ros2 topic list | grep hmi`

### General Issues
**Build failures:**
```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select vision_module monitor_cpp
```

**Package not found:**
```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Or add to ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Development

### Running Tests
```bash
cd ~/ros2_ws
colcon test --packages-select vision_module monitor_cpp
colcon test-result --verbose
```

### Debugging
Enable detailed logging:
```bash
ros2 run vision_module vision_node --ros-args --log-level debug
```

## Documentation

- [HMI Package Documentation](hmi_pkg/hmi_pkg_readme.md) - Detailed HMI setup and usage
- [Vision Module Documentation](vision_module/README.md) - Vision system details
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/) - ROS 2 reference
- [MoveIt 2 Docs](https://moveit.picknik.ai/main/index.html) - Motion planning reference

## License

MIT License - See LICENSE file for details

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Acknowledgments

- OpenAI Whisper for speech recognition
- Ultralytics YOLO for object detection
- MoveIt for motion planning
- Franka Emika for the robot platform
- ROS 2 community
