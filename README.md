# ROS-Maude Bridge  
**Bridge ROS2 and Maude for formal verification and simulation of robotic systems.**  

## Overview  
This project enables bidirectional interaction between ROS2 nodes and Maude objects, allowing:  
- **Real ROS2 Integration**: Maude objects to create ROS2 publishers/subscriptions via `rclpy`.  
- **Pure Maude Simulation**: Simulate ROS2-like behavior entirely within Maude (no ROS2 dependency).  
- **Interoperability**: Maude-based nodes communicate with standard ROS2 nodes.  

## Features  
- Dual-layer architecture:  
  - **External Layer**: `ros_external.maude` connects to ROS2 via `rosmaude.py`.  
  - **Logical Layer**: `ros_logical.maude` simulates ROS2 behavior in pure Maude.  
- Message type translation (currently only `str`(contructable from maude String) ↔ `std_msgs.msg.String`).  
- Simulation mode for testing without ROS2.  

## enviroment  
- **ROS2 jazzy** (or newer) installed and sourced.  
- **Python 3.12+** with `rclpy` and `maude`.  

## try this out 
1. Clone this repository:  
   ```bash  
   git https://github.com/rjsun06/rosmaude
   cd rosmaude
2.
   ```bash
   python3 rosmaude.py \[--simulation\] test_ros.maude
