# swarm-autonomy-simulation
This repository contains code for simulating the MCU autonomy related software. It includes C++ code to run simulations and python code to visualize results.

## state_estimator
- sensor_benchmarker.py: Computes the uncertainty of different sensor configurations for state estimation.
- sensor_specs.json: Specifications for various sensors used in the benchmarking.
- state_estimator.py: Simulates state estimation using an Extended Kalman Filter (EKF) and Rauch-Tung-Striebel (RTS) smoother.