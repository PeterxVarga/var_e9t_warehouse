# Warehouse robot route optimization with ROS 2

Order-picking route optimization and replanning for a simulated warehouse mobile robot.

## Goal

The robot completes pickup jobs on a known aisle network and returns to the depot. The project examines route length and replanning after an aisle closure. Route length and robot execution time are evaluated separately.

## Planned features

- Warehouse layout, depot, and pickup jobs with unique identifiers.
- Comparison of FIFO, nearest-neighbor, and 2-opt pickup sequences.
- Robot motion and job execution with ROS 2 state feedback.
- Replanning from the current robot position after an aisle closure.
- Warehouse visualization, routes, job states, and measurement logs.
- Exact reference solutions for small cases and comparative evaluation for larger cases.

## Status

The project is in the planning stage. The ROS 2 implementation, run instructions, and experimental results will be added as development progresses.

## Environment

Planned ROS 2 distribution: Humble. The simulation environment and dependencies will be documented after the initial technical trial.
