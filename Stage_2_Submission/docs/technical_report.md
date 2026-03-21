# Technical Report: Advanced Driver Assistance System (ADAS) on CARLA Simulator

---

**Team Name:** *[Your Team Name]*  
**Institution:** *[Your Institution]*  
**Date:** March 2026  

---

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [System Overview](#2-system-overview)
3. [Methodology](#3-methodology)
4. [Algorithms & Calculations](#4-algorithms--calculations)
5. [Architecture](#5-architecture)
6. [Results & Observations](#6-results--observations)
7. [Challenges & Future Scope](#7-challenges--future-scope)
8. [References](#8-references)

---

## 1. Problem Statement

### 1.1 Background

Road traffic accidents remain one of the leading causes of fatalities worldwide, with the World Health Organization (WHO) reporting approximately 1.35 million deaths annually. A significant proportion of these accidents — estimated at over 90% — are caused by human error, including distracted driving, delayed reaction times, and misjudgment of distances. Advanced Driver Assistance Systems (ADAS) have emerged as a critical technology layer designed to augment human driving capabilities, reduce accident severity, and pave the way toward fully autonomous vehicles.

### 1.2 Problem Definition

The objective of this project is to design, implement, and demonstrate a **comprehensive ADAS stack** that operates in real-time within a high-fidelity driving simulator. The system must:

1. **Perceive** the driving environment through multiple sensor modalities (camera, LiDAR, GNSS, collision, lane invasion)
2. **Detect and classify** objects in the vehicle's path (vehicles, pedestrians, traffic lights)
3. **Assess risk** by computing Time-to-Collision (TTC) and classifying threat levels
4. **Act** through Forward Collision Warning (FCW) and Automatic Emergency Braking (AEB) interventions
5. **Navigate** autonomously using a behavior-based planning agent with parking capability
6. **Visualize** all system states through a real-time Heads-Up Display (HUD) dashboard

### 1.3 Scope

The system is built on the **CARLA Simulator (v0.9.13+)**, an open-source autonomous driving research platform based on Unreal Engine 4. CARLA provides photorealistic rendering, accurate vehicle physics, comprehensive sensor simulation, and a rich Python API — making it an ideal testbed for ADAS algorithm development without the risks and costs of real-world testing.

---

## 2. System Overview

### 2.1 Platform

| Component | Technology |
|-----------|-----------|
| **Simulator** | CARLA 0.9.13+ (Unreal Engine 4) |
| **Language** | Python 3.7–3.10 |
| **Rendering** | pygame (HUD & overlays) |
| **Computation** | NumPy (matrix operations, projections) |
| **Agent Framework** | CARLA PythonAPI `BehaviorAgent` |

### 2.2 Implemented ADAS Features

The system implements 14 distinct ADAS features organized into four functional layers:

**Perception Layer:**
- Camera RGB feed with real-time rendering
- LiDAR point cloud visualization
- GNSS-based positioning
- Collision detection with impulse intensity logging
- Lane invasion detection with marking type identification

**Detection & Classification Layer:**
- Lane boundary detection via waypoint-to-camera projection
- 3D-to-2D bounding box rendering for all detected actors
- Object classification (Vehicle / Pedestrian / Traffic Light)
- Traffic light state detection (Red / Yellow / Green)
- Distance-based color coding for threat proximity

**Risk Assessment Layer:**
- Time-to-Collision (TTC) continuous estimation
- Three-tier risk classification (Critical / High / Low)
- Forward Collision Warning (FCW) alert generation
- Automatic Emergency Braking (AEB) decision logic

**Control & Planning Layer:**
- Behavior-based autonomous cruise with dynamic re-routing
- Autonomous parallel parking with indicator signals
- Priority-based arbitration for conflicting ADAS commands

### 2.3 Traffic Environment

The simulator spawns a dynamic traffic environment with:
- **15 AI-controlled vehicles** with realistic driving behaviors
- **15 pedestrian walkers** with randomized navigation paths
- Dynamic weather conditions (selectable at runtime)
- Multiple sensor views (3rd-person, hood-mount, side-view, top-down)

---

## 3. Methodology

### 3.1 Development Approach

The project follows a **modular, layered architecture** where each ADAS feature is implemented as an independent module that communicates through well-defined interfaces. This approach enables:

- **Independent testing** of each module
- **Easy extension** with new features
- **Clear separation** between perception, decision, and action

### 3.2 Sensor Configuration

| Sensor | Specification | Purpose |
|--------|--------------|---------|
| RGB Camera | 1280×720, 90° FOV, SpringArm mount | Primary perception, lane/object detection |
| LiDAR | 50m range, ray-cast | 3D environmental mapping |
| GNSS | 1Hz update | Global positioning |
| Collision | Event-triggered | Accident detection & logging |
| Lane Invasion | Event-triggered | Unintended lane departure alerts |

### 3.3 Synchronous Simulation

The system operates in **synchronous mode** (`fixed_delta_seconds = 0.05`, i.e., 20 simulation Hz), ensuring:
- Zero-lag between Python processing and Unreal Engine physics
- Deterministic behavior for reproducible testing
- Consistent TTC calculations regardless of system performance

### 3.4 Coordinate Systems

Three coordinate frames are used throughout the system:

1. **World Frame (CARLA):** Right-handed, X-forward, Y-right, Z-up
2. **Camera Frame:** Origin at camera sensor, Z-forward into the scene
3. **Image Frame:** 2D pixel coordinates with origin at top-left corner

Transformations between frames use 4×4 homogeneous matrices obtained from CARLA's actor transforms.

---

## 4. Algorithms & Calculations

### 4.1 Time-to-Collision (TTC) Estimation

**Purpose:** Predict the time remaining before a collision occurs between the ego vehicle and a forward obstacle.

**Algorithm:**

```
Input: Ego vehicle transform, target vehicle transform, ego velocity, target velocity
Output: TTC in seconds (float)

1. Compute inverse transform matrix of ego vehicle
2. Transform target position to ego's local frame → p = [p_x, p_y, p_z, 1]
3. Check conditions:
   a. p_x > 0 (target is ahead)
   b. |p_y| < 2.0m (target is in-lane)
   c. Actor is a vehicle type
4. Compute relative velocity:
   V_rel = |V_ego| - |V_target|
5. If V_rel > 0.1 m/s:
   TTC = p_x / V_rel
6. Return min(TTC) across all qualifying targets
```

**Mathematical Formulation:**

$$TTC = \frac{d_{longitudinal}}{v_{ego} - v_{target}}$$

Where:
- $d_{longitudinal}$ = Forward distance in ego vehicle's local frame
- $v_{ego}$ = Ego vehicle speed (magnitude of velocity vector)
- $v_{target}$ = Target vehicle speed (magnitude of velocity vector)

### 4.2 Camera Projection (Pinhole Model)

**Intrinsic Matrix Construction:**

```
f = width / (2 × tan(FOV/2))

K = | f    0    w/2 |
    | 0    f    h/2 |
    | 0    0     1  |
```

For resolution 1280×720 and FOV = 90°:
- `f = 1280 / (2 × tan(45°)) = 1280 / 2 = 640`
- `c_x = 640, c_y = 360`

**3D-to-2D Projection Pipeline:**

```
1. P_world = [x, y, z, 1]ᵀ                    (World coordinates)
2. P_camera = W2C × P_world                    (Camera coordinates)
3. P_image = K × P_camera[0:3]                 (Image coordinates)
4. p_pixel = (P_image[0]/P_image[2], P_image[1]/P_image[2])  (Normalize)
```

### 4.3 3D Bounding Box Computation

For each detected actor, the 8 corner points of its bounding box are computed:

```
corners[i] = (±extent.x, ±extent.y, ±extent.z, 1)  for i ∈ [0, 7]

World_corners = Actor_Transform × corners
Camera_corners = W2C × World_corners
Image_corners = K × Camera_corners[:, 0:3]
```

The 2D bounding rectangle is the axis-aligned bounding box of the projected corners:
```
x_min, y_min = min(Image_corners[:, 0:2])
x_max, y_max = max(Image_corners[:, 0:2])
```

### 4.4 Lane Boundary Detection

**Waypoint-based Lane Boundary Algorithm:**

```
1. Get current waypoint W₀ at ego vehicle location
2. For i = 0 to 14:
   a. Record waypoint Wᵢ
   b. Advance: Wᵢ₊₁ = Wᵢ.next(2.0m)
3. For each waypoint Wᵢ:
   a. Left_boundary = Wᵢ.location − right_vector × (lane_width / 2)
   b. Right_boundary = Wᵢ.location + right_vector × (lane_width / 2)
   c. Lower Z by 0.5m (compensate for road surface offset)
4. Project all boundary points to image space
5. Render filled polygon + boundary lines
```

### 4.5 Risk Classification & Arbitration

**Decision Tree:**

```
IF min_TTC < 3.0s:
    → CRITICAL RISK: AEB INTERVENTION (override all controls, brake = 1.0)
ELIF min_TTC < 5.0s:
    → HIGH RISK: FCW ACTIVE (visual/auditory warning)
ELIF parking_mode:
    → Execute parking controller
ELSE:
    → NORMAL CRUISE: Behavior Agent drives
```

**Priority Arbitration Table:**

| Priority | Feature | Control Authority |
|----------|---------|-------------------|
| 1 (Highest) | AEB | Full brake override |
| 2 | FCW | Alert only (no override) |
| 3 | Parking Controller | Steer + brake |
| 4 (Lowest) | Cruise Agent | Full control |

---

## 5. Architecture

### 5.1 System Architecture Diagram

The system follows a **layered pipeline architecture** with six major subsystems:

```
┌─────────────────────────────────────────────────────────┐
│                 CARLA SIMULATOR (UE4)                    │
│  ┌──────────┐  ┌──────────┐  ┌────────────────────┐    │
│  │ Physics   │  │ HD Map   │  │ NPC Traffic        │    │
│  │ Engine    │  │ Waypoints│  │ (15 cars + 15 peds)│    │
│  └──────────┘  └──────────┘  └────────────────────┘    │
└───────────────────────┬─────────────────────────────────┘
                        │ Sensor Data
┌───────────────────────▼─────────────────────────────────┐
│                    SENSOR LAYER                          │
│  Camera RGB │ LiDAR │ GNSS │ Collision │ Lane Invasion  │
└───────────────────────┬─────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────┐
│                  PERCEPTION LAYER                        │
│  Lane Detection │ 3D BBox Projection │ Classification   │
│  Traffic Light  │ Object Tracking                       │
└───────────────────────┬─────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────┐
│               RISK ASSESSMENT LAYER                      │
│  TTC Calculator │ Risk Classifier │ AEB/FCW Decision    │
└───────────────────────┬─────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────┐
│           PLANNING & ARBITRATION LAYER                   │
│  Behavior Agent │ Parking FSM │ Priority Arbiter        │
└───────────────────────┬─────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────┐
│                  ACTUATION LAYER                         │
│  Throttle │ Brake │ Steer │ Gear │ Light/Indicator      │
└───────────────────────┬─────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────┐
│                VISUALIZATION LAYER                       │
│  HUD Dashboard │ Lane Overlay │ BBox Overlay │ Alerts   │
└─────────────────────────────────────────────────────────┘
```

### 5.2 Class Structure

| Class | Responsibility | Key Methods |
|-------|---------------|-------------|
| `World` | Scene management, ADAS rendering | `_render_adas()`, `tick()`, `render()` |
| `HUD` | Dashboard display, info panel | `tick()`, `render()`, `notification()` |
| `CameraManager` | Sensor management, image parsing | `set_sensor()`, `_parse_image()` |
| `CollisionSensor` | Collision event handling | `get_collision_history()` |
| `LaneInvasionSensor` | Lane departure event handling | `_on_invasion()` |
| `GnssSensor` | GPS coordinate tracking | `_on_gnss_event()` |
| `KeyboardControl` | User input handling | `parse_events()` |
| `BehaviorAgent` | Autonomous navigation (CARLA API) | `run_step()`, `set_destination()` |

### 5.3 Data Flow

1. **CARLA → Sensors:** The simulator provides raw sensor data each tick (camera frames, LiDAR point clouds, GNSS coordinates, collision events)
2. **Sensors → Perception:** Raw data is processed: camera images are rendered, 3D bounding boxes are computed and projected to 2D, lane waypoints are queried from the HD map
3. **Perception → Risk:** Detected objects' positions and velocities feed the TTC calculator, which classifies risk levels
4. **Risk → Planning:** AEB/FCW decisions influence the planning arbiter, which selects between cruise, parking, and emergency control modes
5. **Planning → Actuation:** The selected controller generates throttle/brake/steer commands applied to the vehicle
6. **All Layers → HUD:** Every layer reports its current state to the HUD for real-time dashboard visualization

---

## 6. Results & Observations

### 6.1 ADAS Feature Verification

| Feature | Status | Observation |
|---------|--------|-------------|
| Lane Detection | ✅ Working | Green polygon overlays correctly track lane boundaries for ~30m ahead |
| Object Detection | ✅ Working | 3D bounding boxes rendered for vehicles, pedestrians, and traffic lights within 80m |
| Distance Color Coding | ✅ Working | Smooth color transition: Green (>20m) → Yellow (10-20m) → Red (<10m) |
| TTC Calculation | ✅ Working | Accurate real-time computation for in-path vehicles with V_rel > 0.1 m/s |
| FCW (Warning) | ✅ Working | Orange "HIGH RISK - FCW ACTIVE" alert displayed when 3.0s < TTC < 5.0s |
| AEB (Braking) | ✅ Working | Red "CRITICAL RISK - AEB INTERVENTION" with automatic braking at TTC < 3.0s |
| Autonomous Cruise | ✅ Working | Behavior agent navigates to random destinations with obstacle avoidance |
| Parking Mode | ✅ Working | Right-lane parking with blinker signals and timed completion |
| Traffic Light Detection | ✅ Working | Red and yellow lights highlighted with colored bounding boxes |
| Collision Logging | ✅ Working | Collision history graph displayed in HUD with impulse intensity |
| Lane Invasion | ✅ Working | Real-time notification on crossing lane markings |
| GNSS Tracking | ✅ Working | Continuous latitude/longitude readout in HUD |
| HUD Dashboard | ✅ Working | Comprehensive info panel with speed, heading, controls, nearby vehicles |
| Traffic Simulation | ✅ Working | 15 vehicles + 15 pedestrians with realistic AI behaviors |

### 6.2 Performance Metrics

| Metric | Value |
|--------|-------|
| **Simulation Frame Rate** | 20 FPS (fixed-step synchronous mode) |
| **Detection Range** | 80 meters |
| **Lane Lookahead** | ~30 meters (15 waypoints × 2m spacing) |
| **TTC Update Rate** | Every simulation tick (50ms) |
| **Bounding Box Accuracy** | Sub-pixel projection accuracy using CARLA ground truth |
| **Parking Completion Time** | ~7.5 seconds (150 ticks × 50ms) |

### 6.3 Key Observations

1. **TTC accuracy**: Since object positions come from CARLA's ground truth (not noisy sensor data), TTC calculations are highly accurate. In a real-world deployment, Kalman filtering would be needed to smooth noisy detections.

2. **Synchronous mode is critical**: Running in asynchronous mode caused erratic TTC values due to variable frame timing. The fixed 50ms timestep ensures consistent physics-aligned computations.

3. **Lane visualization stability**: Using waypoint-based lane detection (rather than vision-based) provides perfectly stable lane overlays even in challenging conditions (rain, fog, night). This demonstrates the value of HD map integration.

4. **Multi-object arbitration**: The minimum-TTC approach effectively handles scenarios with multiple approaching vehicles — the most imminent threat always takes priority.

5. **Traffic density impact**: With 15 NPC vehicles, the system regularly encounters scenarios requiring FCW and AEB activation, providing robust testing of the risk assessment pipeline.

---

## 7. Challenges & Future Scope

### 7.1 Challenges Faced

| Challenge | Solution |
|-----------|----------|
| **Synchronization lag** between Python and UE4 | Forced synchronous mode with fixed timestep (0.05s) |
| **Camera projection artifacts** for objects at image edges | Clamped bounding boxes to display bounds; filtered points behind camera |
| **TTC noise** at very low relative velocities | Added 0.1 m/s minimum threshold to prevent division instability |
| **Parking accuracy** — finding actual parking slots | Used right-lane + forward offset heuristic; future work could use slot detection |
| **HUD readability** at high speeds | Used monospace fonts with shadow backgrounds for contrast |
| **NPC vehicle variety** — some blueprints not suitable for traffic | Filtered to 4-wheel vehicles from 13 major car manufacturers only |

### 7.2 Future Scope

1. **Vision-Based Perception**: Replace ground-truth detection with YOLOv8 or similar deep learning models for camera-based object detection, making the system transferable to real vehicles.

2. **Sensor Fusion**: Integrate camera + LiDAR data using Extended Kalman Filter (EKF) for more robust object tracking in occluded or adverse conditions.

3. **Predictive Path Planning**: Implement trajectory prediction for surrounding vehicles using LSTM or polynomial fitting to anticipate cut-in and lane-change maneuvers.

4. **Adaptive Cruise Control (ACC)**: Extend the behavior agent with longitudinal velocity control to maintain safe following distances automatically.

5. **Blind Spot Monitoring**: Add side-facing sensors and render blind-spot indicators on the HUD for lane-change safety.

6. **Driver Monitoring System (DMS)**: Use webcam-based gaze tracking and drowsiness detection to alert inattentive drivers.

7. **V2X Communication**: Simulate Vehicle-to-Everything (V2X) protocols for cooperative awareness with infrastructure and other vehicles.

8. **Weather-Adaptive Thresholds**: Dynamically adjust TTC thresholds based on road conditions (wet roads → increase AEB threshold from 3.0s to 4.0s).

9. **Reinforcement Learning**: Train an RL-based parking agent that can handle complex parking geometries (perpendicular, angled, multi-point turns).

10. **Real Vehicle Deployment**: Port the algorithms to ROS2 for deployment on actual vehicles with real sensor hardware.

---

## 8. References

1. Dosovitskiy, A., Ros, G., Codevilla, F., López, A., & Koltun, V. (2017). "CARLA: An Open Urban Driving Simulator." *Proceedings of Machine Learning Research*, 78, 1-16.

2. Euro NCAP. (2023). "Assessment Protocol — Safety Assist." European New Car Assessment Programme.

3. ISO 22839:2013. "Intelligent Transport Systems — Forward Vehicle Collision Mitigation Systems — Operation, Performance, and Verification Requirements."

4. Hartley, R., & Zisserman, A. (2003). *Multiple View Geometry in Computer Vision*. Cambridge University Press.

5. CARLA Simulator Documentation. https://carla.readthedocs.io/en/latest/

6. pygame Documentation. https://www.pygame.org/docs/

7. NumPy Documentation. https://numpy.org/doc/

8. SAE International. (2021). "SAE J3016: Taxonomy and Definitions for Terms Related to Driving Automation Systems for On-Road Motor Vehicles."

9. World Health Organization. (2023). "Global Status Report on Road Safety."

10. Janai, J., Güney, F., Behl, A., & Geiger, A. (2020). "Computer Vision for Autonomous Vehicles: Problems, Datasets and State of the Art." *Foundations and Trends in Computer Graphics and Vision*, 12(1-3), 1-308.

---

*End of Technical Report*
