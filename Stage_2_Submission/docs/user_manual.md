# 📘 User Manual

**ADAS Simulator — Complete User Guide**

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [Keyboard Controls](#2-keyboard-controls)
3. [Understanding the HUD Dashboard](#3-understanding-the-hud-dashboard)
4. [ADAS Features Guide](#4-adas-features-guide)
5. [Camera Views](#5-camera-views)
6. [Weather Control](#6-weather-control)
7. [Driving Modes](#7-driving-modes)
8. [ADAS Overlay Guide](#8-adas-overlay-guide)
9. [FAQ](#9-faq)

---

## 1. Quick Start

### Step 1: Start CARLA Server
```bash
# Windows
CarlaUE4.exe

# Linux
./CarlaUE4.sh
```

### Step 2: Launch ADAS System
```bash
python autonomous_system.py --res 1280x720 --behavior normal --loop
```

### Step 3: Observe
- The vehicle begins **autonomous cruise** mode immediately.
- **15 AI vehicles** and **15 pedestrians** are spawned automatically.
- Press `Q` to toggle the full ADAS overlay (bounding boxes, lane lines, TTC).

---

## 2. Keyboard Controls

### Primary Controls

| Key | Action | Description |
|-----|--------|-------------|
| **`Q`** | Toggle ADAS Overlay | Shows/hides lane detection, bounding boxes, TTC, alerts |
| **`P`** | Toggle Parking Mode | Initiates or exits autonomous parallel parking |
| **`TAB`** | Switch Camera | Cycles through available camera viewpoints |
| **`` ` ``** | Cycle Sensor | Switches between RGB → Depth → Segmentation → LiDAR views |
| **`C`** | Change Weather | Cycles forward through weather presets |
| **`Shift+C`** | Reverse Weather | Cycles backward through weather presets |
| **`ESC`** | Quit | Exits the simulator and cleans up actors |

### Display Controls

| Key | Action |
|-----|--------|
| **`H`** | Toggle Help (show/hide controls info) |
| **`R`** | Toggle Recording |
| **`F1`** | Toggle HUD visibility |

---

## 3. Understanding the HUD Dashboard

The Heads-Up Display (HUD) provides real-time information in several sections:

### Top Bar — Vehicle Info
```
Speed: 42.5 km/h | Heading: N 15.3° | Loc: (123.4, 567.8)
```
- **Speed**: Current vehicle speed in km/h
- **Heading**: Compass direction (N/S/E/W) with degree offset
- **Location**: GNSS coordinates (latitude, longitude)

### Left Panel — System Status
```
Simulation: Town03 | Weather: ClearNoon
Vehicles: 15 active | Pedestrians: 15 active
Frame: 12345 | Time: 617.3s
```

### Right Panel — Vehicle Controls
```
Throttle: 0.65 | Brake: 0.00 | Steer: -0.12
Gear: 4 | Mode: CRUISE
```

### Bottom — Collision History
- Bar graph showing collision impulse intensity over the last 5 seconds
- Updates in real-time when collisions occur

### Bottom-Right — ADAS Feature Status Panel (Enhanced Version)
```
═══ ADAS FEATURE STATUS ═══
● Lane Boundary Detection: ACTIVE
● Traffic Lights: 2×RED, 1×GREEN
● Traffic Signs: STOP SIGN, SPEED 60
● Vehicles Detected: 5
● Pedestrians: 3
● TTC: 4.2s | Risk: HIGH
● Collision Sensor: ACTIVE
● Lane Invasion Sensor: ACTIVE
● GNSS Positioning: ACTIVE
```

---

## 4. ADAS Features Guide

### 4.1 Lane Boundary Detection
- **Visual**: Green filled polygon covering the current driving lane
- **Lines**: Green solid lines on left/right boundaries, yellow dashed center line
- **Label**: "LANE BOUNDARY DETECTED" at the top of the overlay
- **Range**: ~30 meters ahead (15 waypoints × 2m spacing)

### 4.2 Forward Collision Warning (FCW)
- **Trigger**: When TTC is between 3.0s and 5.0s
- **Visual**: Orange "HIGH RISK - FCW ACTIVE" banner on screen
- **Action**: Warning only — no vehicle control override

### 4.3 Automatic Emergency Braking (AEB)
- **Trigger**: When TTC falls below 3.0s
- **Visual**: Red "CRITICAL RISK - AEB INTERVENTION" banner
- **Action**: Full braking override (brake = 1.0), overrides all other controls

### 4.4 Object Detection & Classification
- **Detection Range**: Up to 80 meters
- **Classifications**:
  - `[Car]` — Vehicles (color-coded by distance)
  - `[Ped]` — Pedestrians (orange)
  - `[RED/YLW/GRN LIGHT]` — Traffic lights (color matches state)
  - `[STOP SIGN]`, `[SPEED XX]`, `[YIELD]` — Traffic signs (enhanced version)
- **Color Coding by Distance**:
  - 🟢 Green: > 20 meters (safe)
  - 🟡 Yellow: 10–20 meters (caution)
  - 🔴 Red: < 10 meters (danger)

### 4.5 Traffic Light Detection
- Automatically detects traffic lights within 80 meters
- Bounding box matches the light's current state color
- Labels indicate state: `[RED LIGHT]`, `[YLW LIGHT]`, `[GRN LIGHT]`
- Distance shown in meters

### 4.6 Traffic Sign Recognition (Enhanced)
- Detects stop signs, speed limits, yield signs, and generic signs
- Diamond-shaped markers with distance labels
- Color-coded by sign type (red/white/yellow/blue/gray)
- Detection range: 50 meters

### 4.7 Autonomous Parking
1. Press **`P`** to initiate parking
2. Vehicle activates right blinker and moves toward the right lane
3. Parking completes automatically after ~7.5 seconds
4. Press **`P`** again to resume cruising (left blinker activates)

### 4.8 Collision & Lane Invasion Sensors
- **Collision**: Detects impacts with any actor; logs impulse intensity; shown in history graph
- **Lane Invasion**: Alerts when crossing lane markings; identifies marking type (solid, dashed)

### 4.9 GNSS Positioning
- Real-time latitude/longitude from the GNSS sensor
- Displayed continuously in the HUD top bar

---

## 5. Camera Views

Press **`TAB`** to cycle through camera positions:

| View | Description |
|------|-------------|
| **3rd Person (Default)** | Behind and elevated, showing vehicle and surroundings |
| **Hood Mount** | Front-facing, driver's perspective |
| **Side View** | Lateral view of the vehicle |
| **Top Down** | Bird's eye view from above |

---

## 6. Weather Control

Press **`C`** to cycle through weather presets:

| Preset | Conditions |
|--------|------------|
| ClearNoon | Bright, sunny midday |
| CloudyNoon | Overcast sky |
| WetNoon | Wet roads, light rain |
| WetCloudyNoon | Wet roads, overcast |
| MidRainyNoon | Moderate rain |
| HardRainNoon | Heavy rain, reduced visibility |
| SoftRainNoon | Light drizzle |
| ClearSunset | Sunset lighting |
| CloudySunset | Overcast sunset |
| ClearNight | Night, clear sky |

---

## 7. Driving Modes

Set via command-line argument `--behavior`:

| Mode | Behavior |
|------|----------|
| **`cautious`** | Slow, careful driving; maximum following distance |
| **`normal`** | Balanced speed and safety (recommended default) |
| **`aggressive`** | Faster driving, reduced following distance |

---

## 8. ADAS Overlay Guide

Press **`Q`** to toggle the ADAS overlay. When active, you will see:

| Overlay Element | Description |
|-----------------|-------------|
| **Green Polygon** | Current lane area (lane detection) |
| **Green Lines** | Lane boundary lines (left and right) |
| **Yellow Dashed Line** | Center lane guidance |
| **Colored Rectangles** | 3D bounding boxes around objects |
| **Distance Labels** | Distance in meters on each detected object |
| **TTC Display** | Time-to-collision value on the nearest threat |
| **Risk Alerts** | "FCW ACTIVE" or "AEB INTERVENTION" banners |
| **ADAS Status Panel** | Feature status summary (enhanced version) |

---

## 9. FAQ

**Q: Why is the lane overlay not appearing?**
> Press `Q` to enable the ADAS overlay. Lane detection requires valid waypoints — ensure the vehicle is on a mapped road.

**Q: The system says "AEB INTERVENTION" but I didn't press anything?**
> AEB activates automatically when Time-to-Collision drops below 3.0 seconds. This is a safety feature that overrides manual controls.

**Q: Can I drive manually?**
> The system runs in autonomous cruise mode by default. Keyboard controls are limited to toggling features (P, Q, TAB, C, ESC).

**Q: How do I change the map/town?**
> The map is configured in the CARLA server. Stop the server, then launch with a different map: `CarlaUE4.exe -quality-level=Low -carla-map=/Game/Carla/Maps/Town01`

**Q: Why are bounding boxes flickering?**
> Objects at the edge of the camera FOV may briefly project outside the viewable area. This is normal behavior with pinhole camera projection.

**Q: How do I take screenshots?**
> Use your OS screenshot tool (e.g., `Win+Shift+S` on Windows, `gnome-screenshot` on Linux) while the simulator is running.

---

*Last Updated: March 2026*
