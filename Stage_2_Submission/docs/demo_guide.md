# 🎬 Demo & Presentation Guide

**ADAS Simulator — How to Present & Demonstrate**

---

## Table of Contents

1. [Pre-Demo Checklist](#1-pre-demo-checklist)
2. [Demo Flow Script](#2-demo-flow-script)
3. [Key Feature Demonstrations](#3-key-feature-demonstrations)
4. [Talking Points](#4-talking-points)
5. [Common Questions & Answers](#5-common-questions--answers)
6. [Troubleshooting During Demo](#6-troubleshooting-during-demo)

---

## 1. Pre-Demo Checklist

### Hardware
- [ ] Laptop/PC with **dedicated GPU** (NVIDIA GTX 1060+ recommended)
- [ ] **External monitor** or projector connected (if presenting to audience)
- [ ] Stable **power supply** (CARLA is GPU-intensive)

### Software
- [ ] **CARLA Simulator** installed and tested
- [ ] **Python environment** with pygame and numpy installed
- [ ] **Project files** copied to the demo machine
- [ ] Verify by running the system once before the demo

### Environment
- [ ] Close unnecessary background applications
- [ ] Disable notifications and screensaver
- [ ] Set display resolution to **1280×720** or higher
- [ ] Test audio (if applicable for collision alerts)

### Pre-flight Test
```bash
# 1. Start CARLA
CarlaUE4.exe

# 2. Wait 10 seconds for full map load

# 3. Run ADAS system
python autonomous_system.py --res 1280x720 --behavior normal --loop

# 4. Verify:
#    ✅ Camera feed visible
#    ✅ Lane overlay rendering
#    ✅ Bounding boxes appearing
#    ✅ HUD dashboard active
```

---

## 2. Demo Flow Script

### Recommended Demo Duration: 8–10 minutes

### Phase 1: Introduction (1 min)
> *"This is our ADAS — Advanced Driver Assistance System — built on the CARLA Simulator. It implements 16 real-time features for driver safety and autonomous driving. Let me show you the key features."*

### Phase 2: Launch System (1 min)
1. Show CARLA server running
2. Execute `python autonomous_system.py --res 1280x720 --behavior normal --loop`
3. Point out the immediate visuals: vehicle spawning, traffic environment, dashboard

### Phase 3: Core Features Demo (4–5 min)

#### 3a. Lane Boundary Detection (1 min)
> *"The green polygon overlay shows the detected lane boundaries. These are computed by querying the HD map for waypoints and projecting 3D world coordinates onto the 2D camera view using a pinhole camera model."*
- Point out: green polygon, boundary lines, yellow center line, "LANE BOUNDARY DETECTED" label

#### 3b. Object Detection & Classification (1 min)
> *"Every vehicle, pedestrian, and traffic light within 80 meters is detected, classified, and rendered with a 3D bounding box. Notice the color coding: green for safe distance, yellow for caution, red for danger."*
- Point out: bounding boxes, `[Car]`, `[Ped]`, distance labels, color transitions

#### 3c. Traffic Light Detection (30 sec)
> *"When approaching intersections, our system detects traffic light states — you can see the red light highlighted with a RED LIGHT label."*
- Wait for an intersection; point out colored bounding boxes

#### 3d. TTC, FCW & AEB (1–2 min)
> *"The system continuously computes Time-to-Collision. Watch what happens as we approach a slower vehicle..."*
- **Wait for a natural FCW trigger** (or observe traffic)
> *"At TTC below 5 seconds, a Forward Collision Warning appears. If TTC drops below 3 seconds, AEB — Automatic Emergency Braking — takes over with full braking."*

#### 3e. Autonomous Parking (1 min)
> *"Press P to activate autonomous parking. The vehicle signals right and navigates to the parking position."*
1. Press `P`
2. Show the right blinker activating
3. Wait for parking completion (~7.5 seconds)
4. Press `P` again to resume

### Phase 4: Dashboard & Status Panel (1 min)
> *"The HUD shows real-time vehicle data — speed, heading, GNSS location, vehicle controls, and collision history. The ADAS status panel lists all active features."*
- Point to each HUD element

### Phase 5: Additional Features (1 min)
- Toggle weather with `C`: show rain/night effects
- Cycle camera with `TAB`: show different viewpoints
- Toggle ADAS overlay with `Q`: show with/without comparison
- Mention collision sensor and lane invasion sensor

### Phase 6: Wrap-up (30 sec)
> *"In summary, our system implements 16 ADAS features covering perception, risk assessment, and automated control — all running in real-time at 20 FPS. Thank you."*

---

## 3. Key Feature Demonstrations

### Must-Show Features (Core Competition Requirements)

| Feature | How to Show | What to Highlight |
|---------|-------------|-------------------|
| **Lane Boundary Detection** | Drive on any road | Green polygon, boundary lines, label |
| **Traffic Light Detection** | Approach an intersection | Color-coded bbox + state label |
| **Traffic Sign Recognition** | Drive near stop signs | Diamond markers + type labels |

### Impressive Additional Features

| Feature | How to Trigger | Visual Impact |
|---------|---------------|---------------|
| **AEB Activation** | Natural traffic encounter | Red CRITICAL RISK banner + auto-brake |
| **Parking** | Press `P` | Blinker + smooth right-lane parking |
| **Weather Change** | Press `C` | Real-time atmosphere change (rain/night) |
| **ADAS Toggle** | Press `Q` | Dramatic before/after comparison |

---

## 4. Talking Points

### Technical Highlights
- **Pinhole camera model** for accurate 3D-to-2D projection
- **Synchronous mode** ensures deterministic, reproducible results
- **Priority arbitration**: AEB > FCW > Parking > Cruise
- **TTC formula**: `TTC = distance / relative_velocity` with lateral filtering
- **Waypoint-based lane detection** — stable regardless of weather conditions

### Innovation Points
- **16 ADAS features** in a single integrated system
- **Real-time ADAS status panel** for evaluator visibility
- **Multi-level risk classification** (Critical/High/Low)
- **Distance-based color coding** for intuitive threat visualization
- **Ground-truth detection** allows perfect accuracy demonstration

### Competition Differentiators
- All 3 core features (lane, traffic light, traffic sign) clearly labeled on-screen
- Rich dashboard with collision history, nearby vehicles, and control state
- Autonomous parking with indicator signals
- Comprehensive documentation package

---

## 5. Common Questions & Answers

### Q: "Is this using machine learning for detection?"
> *"In this version, we leverage CARLA's ground-truth actor data for perfect detection accuracy, allowing us to focus on the ADAS logic — TTC computation, risk classification, and automated interventions. The architecture is designed so you could swap in a YOLOv8 model for camera-based detection to make it transferable to real vehicles."*

### Q: "How does the AEB decide when to brake?"
> *"The system continuously computes Time-to-Collision using the formula TTC = distance / relative_velocity. When TTC drops below 3.0 seconds, it's classified as CRITICAL and AEB applies full braking, overriding all other controls."*

### Q: "What happens if multiple threats exist?"
> *"The system evaluates TTC for all in-path vehicles and selects the minimum TTC — meaning the most imminent threat. This drives the warning level."*

### Q: "Can this work on a real car?"
> *"The algorithms — TTC, camera projection, lane detection — are the same principles used in production ADAS systems. For real deployment, you'd replace the ground-truth data with sensor-based ML models and port to ROS2."*

### Q: "Why does lane detection work perfectly in rain?"
> *"Because we use waypoint-based detection from CARLA's HD map, not vision-based lane marking detection. This actually demonstrates the advantage of HD map integration for lane keeping."*

### Q: "How many FPS does it run at?"
> *"The simulation runs at 20 FPS in synchronous mode with a fixed 50ms timestep. This ensures consistent physics and TTC calculations regardless of system performance."*

---

## 6. Troubleshooting During Demo

| Issue | Quick Fix |
|-------|-----------|
| CARLA won't start | Restart. Wait 30 seconds. Try: `CarlaUE4.exe -quality-level=Low` |
| Black screen | Wait 3 seconds after launch for first sensor tick |
| No bounding boxes | Press `Q` to enable ADAS overlay |
| Low FPS | Lower resolution: `--res 800x600` |
| Connection refused | Ensure CARLA server is running on port 2000 |
| No traffic | Restart script; traffic spawns during initialization |
| Parking doesn't work | Must be on a multi-lane road with a right lane available |
| ADAS overlay gone | Press `Q` again to toggle it back on |

### Emergency Restart Procedure
```bash
# Kill everything
taskkill /f /im CarlaUE4.exe   # Windows
# kill -9 $(pgrep Carla)       # Linux

# Restart
CarlaUE4.exe
# Wait 10 seconds
python autonomous_system.py --res 1280x720 --behavior normal --loop
```

---

*Last Updated: March 2026*
