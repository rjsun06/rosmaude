# ROS2-Maude Bridge  
**Bridge ROS2 and Maude for formal verification and simulation of robotic systems.**  

## Next Step
Extend this demo into a package.

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
- **ROS2 jazzy** installed and sourced.  
- **Python 3.12+** with `rclpy` and `maude`.  

## Running the Demo 
1. Clone this repository:  
   ```bash  
   git https://github.com/rjsun06/rosmaude
   cd rosmaude
2.
   ```bash
   export MAUDE_LIB=$PWD/rosmaude
3.
   ```bash
   python3 run.py [--simulation] test_ros.maude


Key Files
- `ros_external.maude`: run rosmaude nodes on ros2, allow communication with non-maude ros nodes.
- `ros_logical.maude`: Pure Maude simulation for rosmaude nodes only.
- `rosmaude.py`: Defines the special Hook handler.
- `run.py`: Bridge script implementing special hooks in `ros_external.maude` with `rclpy`.
- `test_ros.maude`: Example Maude configuration.

Supported Message Types
|ROS2 Type | Maude Sorts |
| - | - |
| std_msgs.msg.String | ROSMAUDE#MSG#STRING#DEFAULT$Elt, STRING$String, META-LEVEL$Term |
| std_msgs.msg.Header | ROSMAUDE#MSG#HEADER#DEFAULT$Elt |
| geometry.msg.Point | ROSMAUDE#MSG#GEOMETRY#POINT#DEFAULT$Elt |
| geometry.msg.Point | ROSMAUDE#MSG#GEOMETRY#POINTSTAMPED#DEFAULT$Elt |
| builtin_interfaces.msg.Time | ROSMAUDE#MSG#BUILTIN_INTERFACES#TIME#DEFAULT |
