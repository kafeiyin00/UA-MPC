
# UA-MPC

> **UA-MPC: Uncertainty-Aware Model Predictive Control for Motorized LiDAR Odometry**<br/>
> Jianping Li, Xinhang Xu, Jinxin Liu, Kun Cao, Shenghai Yuan, Lihua Xie<br/>
> Nanyang Technological University, Tongji University<br/>
> IEEE RAL 2025<br/>
> [**Full Paper**](https://ieeexplore.ieee.org/document/10900461) 

## TL;DR
Motorized LiDAR systems can expand the Field of View (FoV) without adding multiple scanners, but existing motorized LiDAR systems often rely on constant-speed motor control, leading to
suboptimal performance in complex environments. To address this, we propose UA-MPC, an uncertainty-aware motor control strategy that balances scanning accuracy and efficiency. Additionally, we develop a ROS-based realistic simulation environment for motorized LiDAR systems, enabling the evaluation of control strategies across diverse scenarios.

## Experiments in simulation (click for video)

[![Simulation](https://img.youtube.com/vi/zkbm0Tkp-PM/maxresdefault.jpg)](https://www.youtube.com/watch?v=zkbm0Tkp-PM)

## Experiments in real scenes

### Digitalize The Hive in 15 mins
[![Simulation](https://img.youtube.com/vi/ocwUdYUv48s/maxresdefault.jpg)](https://www.youtube.com/watch?v=ocwUdYUv48s)

### Digitalize The Spine in 10 Mins

[![Simulation](https://img.youtube.com/vi/1H2dB0aJLSo/maxresdefault.jpg)](https://www.youtube.com/watch?v=1H2dB0aJLSo)

## System Overview

<img src="/fig/coordinates.jpg" alt="Coordinates" width="500px">

<img src="/fig/system.png" alt="System" width="500px">

## Simulation Environment

<img src="/fig/Simulation.png" alt="Simulation" width="500px">


## Acknowlegement

This work is build upon Marsim (https://github.com/hku-mars/MARSIM) and I2EKF (https://github.com/YWL0720/I2EKF-LO). We thank for the disscussion with Dr. Fanze Kong at HKU and Dr. Jie Xu at NTU.
