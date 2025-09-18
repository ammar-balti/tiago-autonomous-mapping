# tiago-autonomous-mapping
This repository contains a Python controller for the Tiago robot in Webots. The robot navigates predefined waypoints around a table while mapping its environment with Lidar. After completing a full loop, it generates a configuration space (c-space) and visualizes it with Matplotlib.

---
## 📢 Disclaimer
This project was developed for educational and portfolio purposes.  
It is based on Webots simulation examples and modified for demonstration.  
Not intended for production or commercial use.


---

## 🚀 Features
- Differential drive control with two wheel motors.
- Waypoint navigation using GPS and Compass for localization.
- Obstacle detection and mapping with Hokuyo URG-04LX Lidar.
- Real-time occupancy-like map displayed on Webots `Display`.
- Configuration space generation using convolution (with `scipy.signal`).
- Visualization of the c-space with Matplotlib.

---

## 🔧 Requirements
- Webots R2023 or later  
- Python 3.8+  
- Python packages (see `requirements.txt`):
  - `numpy`
  - `matplotlib`
  - `scipy`

Install dependencies with:
```bash
pip install -r requirements.txt
```
---

## ▶️ Running the Simulation
Open Webots.
- Load the world file (kitchen.wbt) or create your own world with a Tiago robot.
- Attach the controller:
    - Select the robot → controller field → set to tiago_robot_controller.
- Run the simulation.
- Watch the robot move around the table, build a map, and visualize the c-space.

---

## 📊 Output
- Real-time occupancy-like map drawn on Webots Display.
- Configuration space displayed in a Matplotlib window after completing the loop.
