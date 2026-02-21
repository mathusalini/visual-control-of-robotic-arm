# Visual Control of Robotic Arm

## 📖 Project Overview
This project presents the development of a visual control framework using stereo vision and an Eye-to-Hand camera configuration. By combining a 4-DOF Lynxmotion robotic arm with a Nexus omnidirectional mobile base, we developed a fully autonomous mobile manipulator capable of reliable pick-and-place tasks in dynamic environments.

Conventional robotic manipulators often rely on absolute motion control based on predefined joint positions, making them vulnerable to calibration errors, sensor noise, or unexpected object movements. To address this, our system adopts a Position-Based Visual Servoing (PBVS) approach. It utilizes continuous real-time visual feedback to drive motion control, allowing the robot to adapt to dynamic environments seamlessly.

## ✨ Key Features
* **Eye-to-Hand Visual Servoing:** Real-time feedback-driven control mechanism replacing conventional fixed-path planning, increasing adaptability and precision.
* **Stereo Vision Pipeline:** Developed on a Vpnon miniPC running Ubuntu. Utilizes dual webcams calibrated via the checkerboard method and OpenCV for real-time color detection, depth estimation, and 3D coordinate calculation.
* **Unified Control System:** Seamlessly synchronizes the motion of the omnidirectional mobile base and the 4-DOF manipulator using precise inverse kinematic computations.
* **Dynamic Path Planning:** Integrates the **D* Lite algorithm** to translate visual data into actionable movement, enabling intelligent obstacle avoidance in dynamic settings.
* **Simulation to Reality:** The entire workflow was thoroughly modeled and validated in Webots simulation before being successfully deployed and experimentally verified on the physical hardware.

## ⚙️ Hardware Architecture
* **Mobile Base:**  4WD Nexus Omnidirectional Robot 
* **Manipulator:** 4-DOF Lynxmotion Robotic Arm
* **Vision Sensors:** Dual Webcams (Stereo Vision Setup)
* **Computing Unit:** Vpnon miniPC (running Ubuntu) acting as the high-level brain.
* **Microcontroller:** Arduino for low-level motor actuation, encoder reading, and PID control.

## 💻 Software & Algorithms
* **Computer Vision:** OpenCV (Color segmentation, Stereo Matching, Depth estimation)
* **Control Strategy:** Position-Based Visual Servoing (PBVS)
* **Path Planning:** D* Lite Algorithm for dynamic obstacle-aware navigation
* **Kinematics:** Custom Inverse Kinematics for both the 4-DOF arm and the Mecanum wheel base.
* **Simulation:** Webots Robotics Simulator

## 🚀 System Workflow
1. **Perception:** Dual webcams capture real-time visual data. OpenCV processes this to detect targets (via color/shape) and computes 3D coordinates using stereo disparity.
2. **Coordinate Transformation:** Translates the camera's visual coordinate frame into the robot's local spatial map.
3. **Path Planning & Navigation:** The D* Lite algorithm maps a collision-free path for the Nexus base to approach the target.
4. **Manipulation:** Once in range, the inverse kinematics engine calculates the exact joint angles required for the Lynxmotion arm to execute the pick-and-place task.
5. **Closed-Loop Feedback:** The system continuously updates its position based on live camera feeds, adjusting to any moving obstacles or shifts in the target's location.

```mermaid
graph TD
    A[📸 Perception <br> Dual Webcams & OpenCV] -->|Target & 3D Coords| B[🗺️ Coordinate Transformation <br> Camera Frame to Robot Map]
    B -->|Localized Target| C[🧭 Path Planning & Navigation <br> D* Lite Algorithm]
    C -->|Collision-Free Path| D[🦾 Manipulation <br> Inverse Kinematics for Lynxmotion Arm]
    D -->|Execution| E[🔄 Closed-Loop Feedback <br> Real-Time Visual Updates]
    
    %% Feedback Loop
    E -.->|Continuous Adjustment| A
    
    %% Styling
    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#bbf,stroke:#333,stroke-width:2px
    style C fill:#bfb,stroke:#333,stroke-width:2px
    style D fill:#fbb,stroke:#333,stroke-width:2px
    style E fill:#eee,stroke:#333,stroke-width:2px,stroke-dasharray: 5 5
