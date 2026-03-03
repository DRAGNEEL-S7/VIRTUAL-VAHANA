# 🚗 Virtual Vahana: Level 2+ Advanced Driver Assistance System

[![CARLA Compatible](https://img.shields.io/badge/CARLA-0.9.x-blue.svg)](https://carla.org/)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-brightgreen.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 👥 Team Details
* **Team Name:** [Insert Team Name Here]
* **Institution:** [Insert Institution/University Here]
* **Track:** Virtual Vahana - Round 1
* **Team Members:**
  * **[Member 1 Name]** - [Role, e.g., Perception & Sensor Fusion]
  * **[Member 2 Name]** - [Role, e.g., Control Systems & Kinematics]
  * **[Member 3 Name]** - [Role, e.g., Simulation Integration & HMI]

---

## 📖 Project Overview
Virtual Vahana is an industry-grade, sensor-fused Advanced Driver Assistance System (ADAS) built within the CARLA Simulator. Designed specifically for unpredictable urban environments, the system adheres to a strict **"Human-First, Safety-Override"** philosophy. 

While the human driver maintains primary control over the vehicle, the ADAS continuously runs spatial risk assessments in the background. It utilizes a 1D Kalman Filter to clean raw radar noise and a PID controller for longitudinal stability. If a critical collision vector involving another vehicle or a Vulnerable Road User (VRU/Pedestrian) is detected, the system executes an absolute manual override to safely stop the vehicle.

---

## ✨ Key Features
* **Dynamic Adaptive Cruise Control (ACC):** Utilizes a PID controller and the kinematic "2.2-Second Rule" to maintain a speed-adaptive following distance.
* **Autonomous Emergency Braking (AEB):** Multi-stage braking logic (FCW Pre-charge -> AEB Full Override) based on dynamic Time-to-Collision (TTC) calculations.
* **Vulnerable Road User (VRU) Protection:** Widened radar FOV and geometric altitude filtering specifically calibrated to detect and protect pedestrians in crosswalks.
* **Lane Keeping Assist (LKA):** Proportional autosteer calculation based on waypoint cross-track error that gently assists the driver without locking the steering wheel.
* **Synchronous Simulation Engine:** Custom CARLA configuration synchronizing the World Tick, Traffic Manager, and Python Control Loop to prevent sensor desynchronization and physics engine stuttering at high speeds.
* **Futuristic HMI:** A Pygame-driven dashboard displaying real-time telemetry, radar target distance, dynamic safe gaps, and visual ADAS intervention alerts.

---

## 🛠️ Prerequisites
To run this simulation, you will need the following installed on your machine:
* **CARLA Simulator** (v0.9.13 or newer recommended)
* **Python** (v3.8 or newer)
* **Python Libraries:**
  ```bash
  pip install pygame numpy
