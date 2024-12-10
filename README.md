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

# Links

## Video Links
[Final Project, Implementation Run](https://youtu.be/RwbA1kzdZYU)
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
 - Level 2: Placeholder test for publishing is created

## Phase 2
For Phase 2, the actual implementation of the C++ API and the ROS2 node working with the API is completed.

### Changelogs
 - GoalPublisher1 renamed to GoalGenerator
 - UML Diagrams edited and API changes based on UML diagram-revised

### Features

ROS2 Node: GoalGenerator
 - Uses search and goals library
 - Publishes goals ascynchronously to Nav2 topic using ActionClient
 - Performs object detection simultaeneously by subscribing to camera topic
 - Work with namespaces
 - Requires compulsory parameter for namespace


URDF & SDF
 - Turtlebot URDF & SDF edited to support namespaces

Nav2
 - Added Nav2 launch files and edited to support swift_scout namespaces

RViz
 - Custom RViz file for Nav2 visualization

Launch file
 - Launches as dependencies as a single command
 - Default launches 2 robots, robot number can be changed by user input
 - User required to provide robot spawn location. Spawn points can be hardcodded or passed using a .txt file
 - User required to provide map file for Nav2 localization
    
Unit Tests
 - USed during initial implementation
 - Don't work as intended in final implementation because of addition of compulsory namespace requirement. Namespace is assigned dynamically and node run requires compulsory parameter.
 - Proper unit tests for node can't be written because of Nav2 AMCL and map server running requirements (Computationaly heavy and takes time to launch)

### Run Node
```bash
# Run goal_pub node
  ros2 run swift_scout goal_pub < namespace >

# Example run
  ros2 run swift_scout goal_pub tb1
```

## Launch
```bash
# Launch the main implementation
  ros2 launch swift_scout swift_scout_main.launch.py num:=< add robot num > # If no num:= default robot spawn 2

# Launch with params
  ros2 launch swift_scout swift_scout_main.launch.py num:=< add robot num > < param name >:=< param value >

# View params
  ros2 param list
```

# Future Works
 - Path planning approach for exploration instead of random goal generation
 - Dynamic robot spawn locations

# Build Commands

## How to build

```bash
rm -rf build/ install/
colcon build 
source install/setup.bash
```

## How to build for tests (unit test and integration test)

```bash
rm -rf build/ install/
colcon build --cmake-args -DCOVERAGE=1 
```

## How to run tests (unit and integration)

```bash
source install/setup.bash
colcon test
# View test results
colcon test-result --all --verbose
```

## How to generate coverage reports after running colcon test

First make sure we have run the unit test already.

```bash
colcon test
```

### Test coverage report for `multi_robot`:

``` bash
ros2 run swift_scout generate_coverage_report.bash
open build/swift_scout/test_coverage/index.html
```

### Test coverage report for `sar`:

``` bash
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select sar \
       --cmake-target "test_coverage" \
       --cmake-arg -DUNIT_TEST_ALREADY_RAN=1
open build/sar/test_coverage/index.html
```

### combined test coverage report

``` bash
./do-tests.bash
```

## How to generate project documentation
``` bash
./do-docs.bash
```

