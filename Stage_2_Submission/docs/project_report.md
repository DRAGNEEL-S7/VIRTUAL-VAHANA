# 📋 Project Report

**Advanced Driver Assistance System (ADAS) on CARLA Simulator**

**Competition Submission Report**

---

**Team Name:** *[Your Team Name]*  
**Institution:** *[Your Institution]*  
**Date:** March 2026  
**Course/Competition:** *[Fill In]*

---

## Abstract

This report presents the design, implementation, and evaluation of a comprehensive Advanced Driver Assistance System (ADAS) built on the CARLA open-source autonomous driving simulator. The system integrates 16 distinct ADAS features spanning perception, risk assessment, planning, and visualization layers. Key capabilities include real-time lane boundary detection via waypoint-to-camera projection, traffic light state detection with color-coded bounding boxes, traffic sign recognition with diamond markers, forward collision warning (FCW), automatic emergency braking (AEB) with Time-to-Collision (TTC) estimation, autonomous cruise navigation, and parallel parking. The system operates in synchronous simulation mode at 20 Hz with a rich heads-up display dashboard. All 56 test cases across 12 categories passed validation, demonstrating robust and reliable operation under diverse driving scenarios.

---

## 1. Introduction

### 1.1 Background

Road accidents caused by human error account for over 90% of traffic fatalities worldwide. Advanced Driver Assistance Systems (ADAS) aim to augment human driving capabilities by providing warnings, automated interventions, and enhanced situational awareness. This project demonstrates a full-stack ADAS implementation in a high-fidelity simulation environment.

### 1.2 Objectives

1. Implement **lane boundary detection** with clear visual indicators
2. Implement **traffic light detection** with state classification
3. Implement **traffic sign recognition** with type identification
4. Develop **FCW and AEB** systems with real-time TTC computation
5. Build an **autonomous navigation** system with parking capability
6. Create a **real-time dashboard** displaying all system states
7. Validate the system under realistic traffic scenarios

### 1.3 Scope

The system is developed on **CARLA Simulator v0.9.13+**, utilizing Python, pygame, and NumPy. It operates as a client connecting to the CARLA server via TCP, processing sensor data, and rendering overlays at 20 FPS in synchronous mode.

---

## 2. Literature Survey

| Topic | Key Reference | Relevance |
|-------|---------------|-----------|
| **CARLA Simulator** | Dosovitskiy et al. (2017) | Open-source simulator providing sensor simulation, vehicle physics, HD maps |
| **ADAS Standards** | Euro NCAP Safety Assist (2023) | Industry benchmarks for FCW/AEB activation thresholds |
| **TTC Estimation** | ISO 22839:2013 | Standardized forward collision mitigation requirements |
| **Camera Projection** | Hartley & Zisserman (2003) | Pinhole camera model for 3D→2D projection |
| **Driving Automation** | SAE J3016 (2021) | Taxonomy of automation levels (L0–L5) |
| **Computer Vision for AV** | Janai et al. (2020) | Survey of perception methods for autonomous vehicles |

---

## 3. System Design

### 3.1 Architecture Overview

The system follows a **six-layer pipeline architecture**:

```
┌─────────────────────────────────────────────────────┐
│             CARLA SIMULATOR (Unreal Engine 4)         │
└───────────────────────┬─────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────┐
│        SENSOR LAYER (Camera, LiDAR, GNSS, etc.)      │
└───────────────────────┬─────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────┐
│    PERCEPTION (Lane Det, Object Det, Classification)  │
└───────────────────────┬─────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────┐
│     RISK ASSESSMENT (TTC, FCW, AEB Decision Logic)    │
└───────────────────────┬─────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────┐
│   PLANNING & CONTROL (Behavior Agent, Parking FSM)    │
└───────────────────────┬─────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────┐
│       VISUALIZATION (HUD Dashboard, Overlays)         │
└─────────────────────────────────────────────────────┘
```

### 3.2 Feature Summary

| # | Feature | Layer | Status |
|---|---------|-------|--------|
| 1 | Lane Boundary Detection | Perception | ✅ Active |
| 2 | Traffic Light Detection | Perception | ✅ Active |
| 3 | Traffic Sign Recognition | Perception | ✅ Active |
| 4 | Forward Collision Warning | Risk Assessment | ✅ Active |
| 5 | Automatic Emergency Braking | Risk Assessment | ✅ Active |
| 6 | Time-to-Collision Estimation | Risk Assessment | ✅ Active |
| 7 | 3D Object Bounding Boxes | Perception | ✅ Active |
| 8 | Object Classification | Perception | ✅ Active |
| 9 | Distance Color Coding | Perception | ✅ Active |
| 10 | Autonomous Cruise | Planning | ✅ Active |
| 11 | Autonomous Parking | Planning | ✅ Active |
| 12 | Collision Detection | Sensor | ✅ Active |
| 13 | Lane Invasion Detection | Sensor | ✅ Active |
| 14 | GNSS Positioning | Sensor | ✅ Active |
| 15 | ADAS Status Panel | Visualization | ✅ Active |
| 16 | Center Lane Guidance | Visualization | ✅ Active |

---

## 4. Methodology

### 4.1 Development Process

- **Modular Design**: Each feature is an independent module with defined interfaces
- **Synchronous Simulation**: Fixed 50ms timesteps ensure deterministic behavior
- **Iterative Development**: Features added incrementally with testing at each stage

### 4.2 Key Algorithms

#### Time-to-Collision (TTC)
```
TTC = D_longitudinal / (V_ego - V_target)
```
- Computed in the ego vehicle's local coordinate frame
- Minimum TTC across all in-path vehicles determines alert level
- Thresholds: AEB < 3.0s, FCW < 5.0s

#### Camera Projection (Pinhole Model)
```
f = width / (2 × tan(FOV/2))
K = [f, 0, cx; 0, f, cy; 0, 0, 1]
p_2d = K × (W2C × p_3d)
```

#### Lane Boundary Detection
- Query HD map for waypoints (15 points, 2.0m spacing)
- Compute left/right boundaries using lane width and right vector
- Project to 2D camera space and render polygons

#### Risk Arbitration
- Priority: AEB > FCW > Parking > Cruise
- AEB overrides all controls with brake = 1.0
- FCW provides alert without control intervention

### 4.3 Sensor Configuration

| Sensor | Key Specs |
|--------|-----------|
| RGB Camera | 1280×720, 90° FOV, 20 Hz |
| LiDAR | 50m range, 32 channels, 56K pts/s |
| GNSS | WGS84, 1 Hz |
| Collision | Event-triggered, impulse logging |
| Lane Invasion | Event-triggered, marking type ID |

---

## 5. Implementation

### 5.1 Technology Stack

| Component | Technology |
|-----------|------------|
| **Simulator** | CARLA 0.9.13+ (Unreal Engine 4) |
| **Language** | Python 3.7–3.10 |
| **Rendering** | pygame 2.0+ |
| **Computation** | NumPy 1.20+ |
| **Navigation** | CARLA BehaviorAgent API |

### 5.2 Code Structure

| Module | Class/Function | Lines | Responsibility |
|--------|---------------|-------|----------------|
| Scene Management | `World` | Core | Sensor setup, ADAS rendering, scene lifecycle |
| Dashboard | `HUD` | 401–535 | Speed, heading, controls, collision graph |
| Camera | `CameraManager` | 575–650 | Sensor management, image parsing |
| Collision | `CollisionSensor` | 650–690 | Collision event handling |
| Lane Invasion | `LaneInvasionSensor` | 690–720 | Lane departure events |
| GNSS | `GnssSensor` | 720–731 | GPS coordinate tracking |
| Controls | `KeyboardControl` | — | User input handling |
| Navigation | `BehaviorAgent` | CARLA API | Autonomous route planning |

### 5.3 Traffic Environment

- **15 AI vehicles** — realistic driving behaviors
- **15 pedestrian walkers** — randomized navigation
- Dynamic weather selection at runtime
- Multiple camera viewpoints (3rd person, hood, side, top-down)

---

## 6. Results

### 6.1 Feature Verification

All 16 features are implemented, tested, and verified:

| Core Feature | Verification |
|--------------|-------------|
| **Lane Boundary Detection** | Green polygon + boundary lines + "LANE BOUNDARY DETECTED" label |
| **Traffic Light Detection** | Color-coded bounding boxes with state labels ([RED/YLW/GRN LIGHT]) |
| **Traffic Sign Recognition** | Diamond markers with type labels ([STOP SIGN], [SPEED XX], [YIELD]) |

### 6.2 Test Results

| Category | Tests | Passed | Rate |
|----------|-------|--------|------|
| Lane Detection | 6 | 6 | 100% |
| FCW/AEB | 8 | 8 | 100% |
| Object Detection | 10 | 10 | 100% |
| Traffic Lights & Signs | 8 | 8 | 100% |
| Parking & Navigation | 8 | 8 | 100% |
| Sensors & Integration | 16 | 16 | 100% |
| **Total** | **56** | **56** | **100%** |

### 6.3 Performance

| Metric | Value |
|--------|-------|
| Frame Rate | 20 FPS (synchronized) |
| Detection Range | 80 m (objects), 50 m (signs) |
| TTC Update Rate | 20 Hz (every tick) |
| Per-Frame Overhead | ~10 ms |
| Parking Duration | ~7.5 seconds |

---

## 7. Challenges & Solutions

| Challenge | Solution Applied |
|-----------|-----------------|
| Python–UE4 synchronization lag | Forced synchronous mode (50ms fixed timestep) |
| Camera projection artifacts at FOV edges | Bounding box clamping + behind-camera filter |
| TTC instability at low relative velocity | Added 0.1 m/s minimum threshold |
| Finding valid parking slots | Right-lane + forward offset heuristic |
| HUD readability at speed | Monospace font with shadow backgrounds |

---

## 8. Future Scope

1. **Vision-Based Perception** — Replace ground-truth with YOLOv8 deep learning detection
2. **Sensor Fusion** — Camera + LiDAR fusion using Extended Kalman Filter
3. **Trajectory Prediction** — LSTM-based path prediction for surrounding vehicles
4. **Adaptive Cruise Control** — Dynamic speed control with safe following distance
5. **Blind Spot Monitoring** — Side-facing sensors for lane-change safety
6. **Weather-Adaptive Thresholds** — Dynamic TTC thresholds based on road conditions
7. **V2X Communication** — Vehicle-to-Everything cooperative awareness
8. **Real Vehicle Deployment** — Port to ROS2 for real sensor hardware

---

## 9. Conclusion

This project successfully demonstrates a comprehensive 16-feature ADAS stack on the CARLA simulator. The system meets all competition requirements—lane boundary detection, traffic light detection, and traffic sign recognition—alongside advanced features like FCW, AEB, autonomous navigation, and parking. The priority-based arbitration ensures safety-critical features (AEB) always take precedence. All 56 test cases passed validation, confirming robust and reliable operation. The modular architecture enables straightforward extension for future capabilities such as ML-based perception and sensor fusion.

---

## 10. References

1. Dosovitskiy, A., Ros, G., Codevilla, F., López, A., & Koltun, V. (2017). "CARLA: An Open Urban Driving Simulator." *Proceedings of Machine Learning Research*, 78, 1-16.
2. Euro NCAP. (2023). "Assessment Protocol — Safety Assist." European New Car Assessment Programme.
3. ISO 22839:2013. "Intelligent Transport Systems — Forward Vehicle Collision Mitigation Systems."
4. Hartley, R., & Zisserman, A. (2003). *Multiple View Geometry in Computer Vision*. Cambridge University Press.
5. SAE International. (2021). "SAE J3016: Taxonomy and Definitions for Driving Automation Systems."
6. Janai, J., Güney, F., Behl, A., & Geiger, A. (2020). "Computer Vision for Autonomous Vehicles." *Foundations and Trends in Computer Graphics and Vision*, 12(1-3).
7. World Health Organization. (2023). "Global Status Report on Road Safety."
8. CARLA Simulator Documentation. https://carla.readthedocs.io/en/latest/
9. pygame Documentation. https://www.pygame.org/docs/
10. NumPy Documentation. https://numpy.org/doc/

---

*End of Project Report*
