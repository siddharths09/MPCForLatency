# MPC for Robots under Communication latency and Packet Loss

Aaditya Shrivastava
Kush Patel
Sarvesh Samadhan Vichare
Siddharth Singh

## Problem Overview
Modern Teleoperation and cloud-based solutions face latency issues because of signal drop and network fluctuations. Standard algorithms assume perfect communication protocols and so cause unpredictable command delays. The goal of our project is to use localized MPC to perform actions relative to the current state of the robot in order to make operation smoother, safer and predictable.

## Algorithm:
1. Detect Packet Loss/Latency condition
2. Analyze current state:
   a) Previous command
   b) Sense environment
   c) Model physics
3. Predict waypoints →MPC
4. Implement commands using IK/Dynamics
