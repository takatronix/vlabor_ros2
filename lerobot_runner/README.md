# lerobot_runner

LeRobot policy runner for ROS2 - Run trained policies (SmolVLA, ACT) with WebAPI control.

## Overview

This package provides a ROS2 node that runs LeRobot-trained policies for robot control.
It supports dynamic model loading, task switching, and external control via WebAPI.

## Features

- **Multiple Policy Support**: SmolVLA, ACT (extensible via registry)
- **WebAPI Control**: Load/unload models, start/stop inference, change tasks
- **Real-time Inference**: Configurable inference and publish rates
- **Action Smoothing**: Low-pass filtering for smooth motion
- **Multi-arm Support**: Dual arm and single arm configurations

## Quick Start

```bash
# Build
cd /home/ros2/ros2_ws
colcon build --packages-select lerobot_runner
source install/setup.bash

# Launch
ros2 launch lerobot_runner policy_runner.launch.py preset:=so101_dual_smolvla
```

## WebAPI

Default port: `8083`

### Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/status` | Current status |
| GET | `/api/models` | Available models |
| GET | `/api/policies` | Available policy types |
| POST | `/api/load` | Load a model |
| POST | `/api/unload` | Unload model |
| POST | `/api/start` | Start inference |
| POST | `/api/stop` | Stop inference |
| POST | `/api/task` | Change task |

### Examples

```bash
# Check status
curl http://localhost:8083/api/status

# Load model
curl -X POST http://localhost:8083/api/load \
  -H "Content-Type: application/json" \
  -d '{"checkpoint_path": "/path/to/model", "policy_type": "smolvla"}'

# Start inference
curl -X POST http://localhost:8083/api/start \
  -H "Content-Type: application/json" \
  -d '{"task": "Pick the red cube"}'

# Change task
curl -X POST http://localhost:8083/api/task \
  -H "Content-Type: application/json" \
  -d '{"task": "Place it in the bowl"}'

# Stop inference
curl -X POST http://localhost:8083/api/stop
```

## Configuration

See `config/presets/` for preset configurations.

## Dependencies

- ROS2 Humble
- LeRobot (`pip install lerobot`)
- PyTorch
- aiohttp (for WebAPI)
