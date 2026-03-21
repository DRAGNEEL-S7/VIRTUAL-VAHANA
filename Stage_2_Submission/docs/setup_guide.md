# 🛠 Setup & Installation Guide

**ADAS Simulator on CARLA — Environment Setup**

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [CARLA Simulator Installation](#2-carla-simulator-installation)
3. [Python Environment Setup](#3-python-environment-setup)
4. [Project Setup](#4-project-setup)
5. [Running the Simulator](#5-running-the-simulator)
6. [Troubleshooting](#6-troubleshooting)

---

## 1. System Requirements

### Minimum Hardware

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Intel i5 / AMD Ryzen 5 | Intel i7 / AMD Ryzen 7 |
| **RAM** | 8 GB | 16 GB |
| **GPU** | NVIDIA GTX 1060 (4 GB VRAM) | NVIDIA RTX 2070+ (8 GB VRAM) |
| **Storage** | 30 GB free | 50 GB free (SSD recommended) |
| **OS** | Windows 10 / Ubuntu 18.04 | Windows 10/11 / Ubuntu 20.04 |

### Software Prerequisites

| Software | Version | Download |
|----------|---------|----------|
| **CARLA Simulator** | 0.9.13+ | [github.com/carla-simulator/carla](https://github.com/carla-simulator/carla/releases) |
| **Python** | 3.7 – 3.10 | [python.org](https://www.python.org/downloads/) |
| **pip** | Latest | Included with Python |
| **NVIDIA Drivers** | 470+ | [nvidia.com/drivers](https://www.nvidia.com/Download/index.aspx) |

---

## 2. CARLA Simulator Installation

### Windows

1. **Download** the pre-built CARLA release (`.zip`) from the [CARLA Releases page](https://github.com/carla-simulator/carla/releases).
2. **Extract** the archive to a directory, e.g., `C:\CARLA_0.9.13\`.
3. **Verify** by running:
   ```cmd
   cd C:\CARLA_0.9.13
   CarlaUE4.exe
   ```
   The CARLA simulator window should appear with a default town scene.

### Linux

1. **Download** the release tarball:
   ```bash
   wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz
   ```
2. **Extract**:
   ```bash
   tar -xzf CARLA_0.9.13.tar.gz -C ~/CARLA_0.9.13
   ```
3. **Run**:
   ```bash
   cd ~/CARLA_0.9.13
   ./CarlaUE4.sh
   ```

### CARLA PythonAPI Setup

After installing CARLA, add the Python API to your path:

```bash
# Add CARLA Python API egg file to Python path
export PYTHONPATH=$PYTHONPATH:/path/to/CARLA/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:/path/to/CARLA/PythonAPI/carla
```

On Windows (PowerShell):
```powershell
$env:PYTHONPATH += ";C:\CARLA_0.9.13\PythonAPI\carla\dist\carla-0.9.13-py3.7-win-amd64.egg"
$env:PYTHONPATH += ";C:\CARLA_0.9.13\PythonAPI\carla"
```

---

## 3. Python Environment Setup

### Create Virtual Environment (Recommended)

```bash
# Create virtual environment
python -m venv adas_env

# Activate
# Windows:
adas_env\Scripts\activate
# Linux/Mac:
source adas_env/bin/activate
```

### Install Dependencies

```bash
pip install pygame numpy
```

### Verify Installation

```python
python -c "import pygame; import numpy; print('All dependencies installed successfully!')"
```

### Dependency Details

| Package | Version | Purpose |
|---------|---------|---------|
| **pygame** | 2.0+ | HUD rendering, keyboard input, display management |
| **numpy** | 1.20+ | Matrix operations, camera projection, coordinate transforms |
| **carla** | 0.9.13 | CARLA Python API (provided with CARLA installation) |

---

## 4. Project Setup

### Clone / Copy the Project

```bash
# Clone from repository
git clone https://github.com/<your-username>/adas-simulator.git
cd adas-simulator

# OR copy project files to working directory
cp -r /path/to/carla_autonomous_driving .
```

### Project Directory Structure

```
carla_autonomous_driving/
├── autonomous_system.py           # Main ADAS application
├── README.md                      # Project overview
├── docs/
│   ├── architecture_diagram.md    # System architecture (Mermaid)
│   ├── feature_logic.md           # TTC, thresholds & arbitration
│   ├── technical_report.md        # Comprehensive technical report
│   ├── setup_guide.md             # This file
│   ├── testing_report.md          # Testing & validation results
│   ├── user_manual.md             # User manual & controls
│   ├── sensor_specifications.md   # Sensor configuration details
│   ├── project_report.md          # Formal competition report
│   └── demo_guide.md              # Demo & presentation guide
├── enhanced_submission/
│   ├── autonomous_system.py       # Enhanced version with TSR
│   └── docs/
│       └── expert_response.md     # Expert feedback response
└── screenshots/
    └── *.png                      # Dashboard & UI screenshots
```

---

## 5. Running the Simulator

### Step 1: Start CARLA Server

```bash
# Windows
cd C:\CARLA_0.9.13
CarlaUE4.exe

# Linux
cd ~/CARLA_0.9.13
./CarlaUE4.sh
```

> **Note:** Wait for the CARLA window to fully load before running the ADAS client.

### Step 2: Run the ADAS System

```bash
# Default configuration (1280×720, normal behavior, looping)
python autonomous_system.py --res 1280x720 --behavior normal --loop
```

### Step 3: Verify System Operation

Once running, verify:
- ✅ Camera feed appears with 3rd-person view
- ✅ Green lane polygon renders on the road
- ✅ Bounding boxes appear around vehicles and pedestrians
- ✅ HUD dashboard displays speed, heading, location
- ✅ Press `Q` to toggle ADAS overlay visibility

### Command-Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--host` | `127.0.0.1` | CARLA server host IP |
| `-p, --port` | `2000` | CARLA server port |
| `--res` | `1280x720` | Display resolution (WxH) |
| `-b, --behavior` | `normal` | Driving behavior: `cautious`, `normal`, `aggressive` |
| `-l, --loop` | off | Continuously pick new destinations |
| `-s, --seed` | None | Random seed for reproducibility |
| `--generation` | `2` | Vehicle blueprint generation |

---

## 6. Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| **`ModuleNotFoundError: No module named 'carla'`** | Add CARLA `.egg` file to `PYTHONPATH` (see Section 2) |
| **CARLA server connection refused** | Ensure CARLA is running and port 2000 is not blocked by firewall |
| **Low FPS / lag** | Lower resolution: `--res 800x600`; close background applications |
| **Black camera feed** | Wait 2–3 seconds after starting; the first sensor tick may be empty |
| **Traffic not spawning** | Ensure the CARLA map has loaded fully before launching the script |
| **`pygame.error: No available video device`** | Install or update GPU drivers; ensure display is connected |
| **Parking not working** | Ensure there's a right lane available; parking uses `get_right_lane()` |
| **Synchronous mode issues** | The script sets synchronous mode; don't run other CARLA clients simultaneously |

### Getting Help

- **CARLA Documentation:** [carla.readthedocs.io](https://carla.readthedocs.io)
- **CARLA Discord:** [discord.gg/carla](https://discord.gg/8kqACuC)
- **pygame Documentation:** [pygame.org/docs](https://www.pygame.org/docs/)

---

*Last Updated: March 2026*
