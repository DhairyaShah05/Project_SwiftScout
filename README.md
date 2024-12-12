# SwiftScout

# Project Badges
![CICD Workflow status](https://github.com/DhairyaShah05/Project_SwiftScout/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

[![codecov](https://codecov.io/gh/DhairyaShah05/Project_SwiftScout/graph/badge.svg?token=Y7PV8e48Nf)](https://codecov.io/gh/DhairyaShah05/Project_SwiftScout)

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

# Overview
This repository contains a swarm robotics implementation designed for Search and Rescue (SaR) operations.

## Authors
 - Harsh Senjaliya
 - Dhairya Shah

## Purpose 
The goal is to develop a 5-year R&D roadmap for implementing multi-agent or swarm robotics in search and rescue operations. SaR missions often involve complex, hazardous environments, requiring personnel to locate individuals or objects in distress. Traditional methods depend heavily on human operators, which can be time-consuming, resource-intensive, and risky. Leveraging robotic systems offers a safer, more efficient alternative by enhancing capabilities and mitigating risks associated with SaR missions.

## Assumption
The area map is provided to the robots in advance for navigation.
The object to be located is stationary and does not move during the operation.

## Features
 - A C++ API for autonomous map navigation and object detection by multiple robots.
 - CI/CD integration and code coverage tracking via GitHub.
 - UML and dependency diagrams.
 - Doxygen-generated documentation.

## Constraints
Simulating a large number of robots in a restricted space while ensuring collision avoidance and optimal autonomous path planning is challenging. Runtime performance and memory management depend on system hardware, while communication latency between 20+ robots can vary due to hardware or environmental factors. Adverse weather conditions or remote locations may also degrade communication quality.
# Process
The project follows AIP principles and employs pair programming, with regular role-swapping between the driver and navigator. Initially, the Gazebo environment is set up with a pre-defined world, area map, and spawned robots. The robots autonomously explore the map, searching for the target object. Once located, the robot that finds the object will notify the system of the object's location.

Testing is carried out using GoogleTest for individual components, while system-wide tests are performed iteratively to ensure overall functionality. Integration tests are conducted to evaluate node performance at the global level.


## AIP Document Links
[AIP Google Sheet](https://docs.google.com/spreadsheets/d/1YPUIW5JSqR-EWpY1HwXwIEfF1Ur8AC_8NLE_qPvy1vg/edit?usp=sharing)

[Sprint Notes](https://docs.google.com/document/d/1Fy3jsI4zflh32w8KlTzDQFwrvc6luE-MJg53ZJx5ot4/edit?usp=sharing)

# Dependencies
The project is built within a ROS2 environment for simulation, using C++ with CMake build tools on Ubuntu 22.04. OpenCV, an Apache 2.0-licensed library, is used for image processing and object detection. ROS2 and Turtlebot3, both under Apache 2.0 licenses, provide the framework and simulation environment, respectively. Algorithms involving HSV-based image manipulation with OpenCV are utilized, while GitHub manages version control, CI/CD, and testing.


# Development
## Packages
 - multi_robot: A package created to use the sar API for Search & Rescue operations and Swarm implementation

## Phase 1
For Phase 1, the initial design of UML diagrams and empty implementation with class stubs and placeholder unit tests are created.

### Features
 - search library: C++ library/API for object detection
 - goals library: C++ library/API for autonomous navigation around a map
 - CameraSubscriber: ROS2 node to subscribe to camera topics for object detection, uses search library as dependency
 - GoalPublisher: ROS2 node to publish navigation goal/positions to the robots, uses goals library as dependency

### Unit Tests
 - Level 1: Tests for the C++ API, to check functionality of every class method was created
 - Level 2 (Incomplete): Placeholder test for publishing is created
