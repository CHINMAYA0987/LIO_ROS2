
# 🚀 LiDAR_LIO_ROS2

> 🎯 **LiDAR–Inertial Odometry (LIO) using ROS2 (C++ implementation inspired by FAST-LIO2 & Faster-LIO). This example is for learning purposes only.**

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)  
![Status](https://img.shields.io/badge/Status-Experimental-success?style=for-the-badge)  
![Mapping](https://img.shields.io/badge/Mapping-3D%20LiDAR-orange?style=for-the-badge)

---

## 🧠 Overview

This repository implements a **Tightly Coupled LiDAR–Inertial Odometry (LIO)** system using **ROS2**, designed for **real-time pose estimation and mapping**.

✨ It combines:
- 📡 LiDAR point cloud processing  
- 📊 IMU measurements  
- 🧭 High-frequency state estimation  

---

## 🎥 Demo

<p align="center">
  <img src="https://github.com/user-attachments/assets/d7be0157-a983-4f50-9fd1-ce87a175f1ce" width="700"/>
</p>


## ⚙️ Features

- ⚡ **Real-time LiDAR-Inertial Odometry**  
- 🔗 **Tightly coupled LiDAR + IMU fusion**  
- 🌳 **Efficient map structure using incremental KD-Tree (ikd-Tree)**  
- 📉 **Low drift estimation with iterative optimization**  
- 🧠 **Plane feature extraction & matching**  
- 🔄 **Continuous map update**  
- 🧩 **ROS2 modular architecture**  

---

## 🏗️ Pipeline Overview

The system follows a modern LIO pipeline:

1. **IMU Propagation**
   - Predicts motion using inertial data  
   - Provides initial guess for LiDAR alignment  

2. **LiDAR Processing**
   - Scan undistortion using IMU  
   - Feature extraction (planes / geometric constraints)  

3. **Scan-to-Map Matching**
   - KNN search + plane fitting 
   - Residual computation for optimization  

4. **State Estimation**
   - Iterative update (iEKF-style)  
   - Pose refinement  

5. **Mapping**
   - Incremental updates using ikd-Tree  
   - Local map maintenance  

---

## 🧪 Dataset

Tested on:
- 📦 **MulRan KAIST Dataset**  


---

## ⚙️ Installation




---

## 📚 References & Inspiration

This implementation is inspired by the following state-of-the-art research:

### 🔹 FAST-LIO2

- Xu, Wei et al.  
  *"FAST-LIO2: Fast Direct LiDAR-Inertial Odometry"*  
- IEEE Transactions on Robotics (2022)  

💡 A highly efficient direct LIO framework with tight coupling and fast state estimation.

---

### 🔸 Faster-LIO

- Bai, X. et al.  
  *"Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry"*  
- IEEE Robotics and Automation Letters (2022)  

💡 Improves computational efficiency and reduces system overhead.

---

### 🌳 ikd-Tree

- Cai, Y. et al.  
  *"ikd-Tree: An Incremental K-D Tree for Robotic Applications"*  
- IEEE Robotics and Automation Letters (2022)  

💡 Enables fast incremental map updates and nearest-neighbor search.

---

## 🎓 Suggested Reading

<details>
<summary>📘 Learn the theory behind this project</summary>

- LiDAR–Inertial Odometry fundamentals  
- Kalman filtering & state estimation  
- Point cloud registration (ICP, KNN, plane fitting)  
- Sensor fusion techniques  

</details>

---

## 🧠 Notes

- This project follows a **tightly-coupled LIO approach**  
- Designed for **learning and experimentation**  
- Focuses on **real-time performance and efficiency**  

---

## 📁 Project Structure

---

## 🚧 Future Work

- Loop closure integration  
- Global optimization (pose graph)  
- GPU acceleration  
- Multi-sensor fusion (Camera + LiDAR + IMU)  

---

## 🤝 Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

---

## 📜 License

MIT License
