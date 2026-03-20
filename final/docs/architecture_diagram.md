# ADAS System Architecture Diagram

## High-Level System Architecture

```mermaid
graph TB
    subgraph CARLA["🎮 CARLA Simulator (Unreal Engine 4)"]
        UE4["Physics Engine"]
        MAP["HD Map / Waypoints"]
        NPC["NPC Vehicles & Pedestrians"]
    end

    subgraph SENSORS["📡 Sensor Layer"]
        CAM["Camera RGB<br/>1280×720, 90° FOV"]
        LID["LiDAR Ray-Cast<br/>50m Range"]
        GNSS["GNSS Sensor<br/>Lat/Lon Positioning"]
        COL["Collision Sensor<br/>Impulse Detection"]
        LANE_S["Lane Invasion Sensor<br/>Marking Type Detection"]
    end

    subgraph PERCEPTION["👁 Perception Module"]
        LANE_D["Lane Detection<br/>Waypoint → Camera Projection"]
        OBJ_DET["Object Detection<br/>3D BBox → 2D Projection"]
        OBJ_CLS["Object Classification<br/>Vehicle / Pedestrian / Light"]
        TL_DET["Traffic Light Detection<br/>Red / Yellow / Green State"]
    end

    subgraph RISK["⚠ Risk Assessment"]
        TTC["TTC Calculator<br/>distance ÷ relative_velocity"]
        RISK_CLS["Risk Classifier<br/>Critical / High / Low"]
        AEB["AEB Decision<br/>TTC &lt; 3.0s → Brake"]
        FCW["FCW Decision<br/>TTC &lt; 5.0s → Warning"]
    end

    subgraph PLANNING["🧠 Planning & Control"]
        BHVR["Behavior Agent<br/>Route Planning"]
        ARB["Arbitration Logic<br/>Priority: AEB &gt; FCW &gt; Cruise"]
        PARK["Parking Controller<br/>State Machine + Timer"]
    end

    subgraph ACTUATION["🔧 Vehicle Actuation"]
        CTRL["Vehicle Control<br/>Throttle / Brake / Steer"]
        LIGHT["Light Control<br/>Indicators / Blinkers"]
    end

    subgraph HUD["📊 Dashboard (HUD)"]
        SPEED["Speed / Heading"]
        LOC["Location / GNSS"]
        COL_HIST["Collision History Graph"]
        NEARBY["Nearby Vehicle List"]
        ALERTS["Risk Alerts Overlay"]
        LANE_VIZ["Lane Polygon Overlay"]
        BBOX_VIZ["Bounding Box Overlay"]
        MODE["Active Mode Display"]
    end

    %% Data Flow
    CARLA --> SENSORS
    CAM --> LANE_D
    CAM --> OBJ_DET
    MAP --> LANE_D
    OBJ_DET --> OBJ_CLS
    OBJ_DET --> TL_DET
    OBJ_CLS --> TTC
    TTC --> RISK_CLS
    RISK_CLS --> AEB
    RISK_CLS --> FCW
    AEB --> ARB
    FCW --> ARB
    ARB --> CTRL
    BHVR --> CTRL
    PARK --> CTRL
    CTRL --> LIGHT

    %% HUD connections
    LANE_D --> LANE_VIZ
    OBJ_CLS --> BBOX_VIZ
    RISK_CLS --> ALERTS
    COL --> COL_HIST
    GNSS --> LOC
    CTRL --> SPEED

    %% Styling
    classDef sensor fill:#2196F3,stroke:#1565C0,color:#fff
    classDef perception fill:#9C27B0,stroke:#6A1B9A,color:#fff
    classDef risk fill:#FF9800,stroke:#E65100,color:#fff
    classDef planning fill:#4CAF50,stroke:#2E7D32,color:#fff
    classDef actuation fill:#F44336,stroke:#C62828,color:#fff
    classDef hud fill:#00BCD4,stroke:#00838F,color:#fff

    class CAM,LID,GNSS,COL,LANE_S sensor
    class LANE_D,OBJ_DET,OBJ_CLS,TL_DET perception
    class TTC,RISK_CLS,AEB,FCW risk
    class BHVR,ARB,PARK planning
    class CTRL,LIGHT actuation
    class SPEED,LOC,COL_HIST,NEARBY,ALERTS,LANE_VIZ,BBOX_VIZ,MODE hud
```

---

## Data Flow Summary

| Stage | Input | Processing | Output |
|-------|-------|-----------|--------|
| **Sensing** | CARLA world state | Sensor data acquisition | Raw camera, LiDAR, GNSS, collision, lane events |
| **Perception** | Camera images + Map waypoints | World-to-camera projection, BBox computation | 2D lane boundaries, 2D object bounding boxes, classifications |
| **Risk Assessment** | Object distances + velocities | TTC computation + threshold comparison | Risk level (Critical/High/Low), AEB/FCW decisions |
| **Planning** | Risk decisions + Navigation goal | Behavior agent + arbitration + parking FSM | Desired vehicle control commands |
| **Actuation** | Control commands | Apply throttle/brake/steer | Vehicle motion + indicator lights |
| **Visualization** | All module outputs | HUD rendering via pygame | On-screen dashboard with overlays |

---

## Module-to-Code Mapping

| Architecture Module | Code Class / Function | Lines |
|--------------------|----------------------|-------|
| Sensor Layer | `CollisionSensor`, `LaneInvasionSensor`, `GnssSensor`, `CameraManager` | 575–731 |
| Lane Detection | `World._render_adas()` — lane polygon section | 222–254 |
| Object Detection | `World._render_adas()` — bounding box section | 256–329 |
| TTC / Risk Assessment | `World._render_adas()` — TTC extrapolation | 275–343 |
| Behavior Agent | `BehaviorAgent` (CARLA PythonAPI) | 814 |
| Parking Controller | `game_loop()` — parking logic | 833–857 |
| Vehicle Actuation | `game_loop()` — control application | 864–871 |
| HUD Dashboard | `HUD` class | 401–535 |
| Traffic Spawning | `spawn_traffic()` | 732–786 |
