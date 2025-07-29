# Labyrinth Navigator 2.0

Labyrinth Navigator 2.0 is a maze-solving robot that uses the **Tremaux Algorithm** for efficient pathfinding. The robot autonomously explores and solves mazes by marking visited paths and backtracking when necessary. The system integrates a series of **PID controllers** to optimize robot movement and ensure smooth operation.

## Features
- **Tremaux Algorithm**: Solves the maze by marking visited paths, ensuring the robot avoids revisiting areas and solves the maze in the shortest time.
- **PID Controllers**: 
  - **Motor Control PID**: Equalizes motor encoder readings for synchronized movement.
  - **Path Centering PID**: Balances sensor inputs to keep the robot centered within the maze path.
  - **Wall Proximity PID**: Slows down the robot as it approaches obstacles to ensure safe navigation.

## Files Overview
- **motor_tune.ino**: Implements PID controllers to ensure precise control over the robot’s motors. This includes:
  - Adjusting motor speed to equalize encoder readings.
  - Maintaining the robot's position in the center of the maze path using sensor feedback.
  - Slowing down when a wall is detected to avoid collisions.

- **Tremaux Algorithm Logic**: Implements the algorithm's core functionality, allowing the robot to explore the maze systematically. It marks paths and uses backtracking to ensure efficient traversal.

## Requirements
- A robot with motors, IR or distance sensors, and a compatible microcontroller.
- Development environment for your chosen microcontroller (e.g., Arduino IDE, etc.).

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/ansumanpatro11/nano_navigator.git
