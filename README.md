# Aquila: Autonomous UAV Playground

**A high-fidelity, containerized simulation testbed for developing advanced Reinforcement Learning (SAC) and Factor Graph-based state estimation algorithms for autonomous UAV landing.**

---

## 🚀 Project Overview

Aquila is a research and development initiative designed to bridge the gap between simulation and deployment for autonomous aerial systems. The primary goal is to create a robust "Hardware-in-the-Loop" (HITL) style environment where novel guidance, navigation, and control (GNC) algorithms can be tested safely. 

Currently, the project focuses on solving the **Precision Landing** problem using a dual-node architecture: leveraging **Factor Graphs** for optimal state estimation and **Soft Actor-Critic (SAC)** reinforcement learning for robust control policies in dynamic environments.

## 🏗 System Architecture

The core infrastructure acts as a middleware translation layer, fusing distinct robotics protocols into a unified development environment.

* **Simulation Engine:** PX4 Autopilot (SITL) running in lock-step with Gazebo Harmonic.
* **Middleware:** ROS 2 Humble acting as the central nervous system.
* **Bridge Layer:** Custom MicroXRCE-DDS bridge translating uORB (embedded) topics to DDS (ROS) messages in real-time.
* **Containerization:** Fully Dockerized pipeline optimized for macOS (Apple Silicon), utilizing a shared network namespace to bypass VM network isolation issues and enabling "Headless" server-grade rendering.

## ⚡ Key Technical Highlights

* **Headless macOS Pipeline:** Engineered a solution to run GPU-accelerated Gazebo simulations inside Docker on macOS without a physical display, streaming sensor data (Depth Camera, IMU, GPS, Optical Flow) to ROS 2.
* **Custom Network Namespacing:** Solved complex UDP Multicast discovery issues inherent to Docker-on-Mac by fusing container network stacks, enabling zero-latency communication between the Flight Controller and the Mission Computer.
* **Sensor Fusion Ready:** Successfully integrated and synchronized high-frequency IMU (250Hz) and visual sensor streams, creating the dataset required for Factor Graph optimization.

## 🚧 Roadmap & Active Development

This project is currently under active development. The infrastructure is operational, and work has shifted to the algorithmic implementation of the autonomy stack.

### **Node A: State Estimator (In Progress)**
* **Objective:** Replace standard EKF solutions with a Smoothing and Mapping approach.
* **Tech:** GTSAM (Georgia Tech Smoothing and Mapping library).
* **Method:** Implementing a Fixed-Lag Smoother Factor Graph to fuse high-rate IMU integration with low-rate GPS and Visual Odometry updates, providing a globally consistent state estimate.

### **Node B: Autonomy Agent (In Progress)**
* **Objective:** Train a neural network to land the UAV on non-cooperative targets.
* **Tech:** PyTorch, Stable-Baselines3, Gymnasium.
* **Method:** Wrapping the ROS 2 environment into a custom OpenAI Gym interface to train a Soft Actor-Critic (SAC) agent. The agent optimizes for energy efficiency and landing precision while minimizing control effort.

## 🛠 Tech Stack

* **Robotics:** ROS 2 Humble, PX4 Autopilot, Gazebo Harmonic
* **Algorithms:** GTSAM, Stable-Baselines3 (SAC), Gymnasium
* **Infrastructure:** Docker, Shell Scripting, MicroXRCE-DDS
* **Languages:** Python 3.10+, C++, XML (SDF)

---
*Author: Aaron Pandian*
