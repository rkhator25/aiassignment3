# aiassignment3 rishabh khator se24ucse163


# Path Planning Algorithms for UGV and Road Networks

## Overview

This project implements three important search algorithms used in Artificial Intelligence and Robotics:

1. Dijkstra’s Algorithm (Uniform Cost Search)
2. A* Algorithm for static obstacle navigation
3. Dynamic path planning with changing obstacles

---

## 1. Dijkstra’s Algorithm

* Used for finding the shortest path between cities.
* Expands the node with the minimum cumulative cost.
* Guarantees optimal solution for non-negative weights.

### Input:

* Graph of cities with road distances

### Output:

* Shortest distance from source city to all other cities

---

## 2. A* Algorithm (Static Environment)

* Used for grid-based navigation.
* Combines:

  * g(n): actual cost
  * h(n): heuristic (Manhattan distance)

### Features:

* Grid-based battlefield (20x20 or scalable)
* Random obstacle generation
* Ensures start and goal are reachable

### Measures of Effectiveness:

* Path Length
* Nodes Expanded (approx.)
* Obstacle Density

---

## 3. Dynamic Path Planning

* Handles unknown or moving obstacles.
* Re-plans path when new obstacle appears.

### Approach:

* Repeated A* search (simplified D* concept)
* Detect obstacle → replan → continue

### Real-world relevance:

* Autonomous vehicles
* Military UGV navigation
* Robotics in uncertain environments

---

## Requirements

* Python 3.x
* No external libraries required

---

## How to Run

1. Copy the code into a Python file
2. Run using:
   python filename.py

---

## Notes

* Grid size can be increased (e.g., 70x70)
* Obstacle density can be adjusted (0.1, 0.2, 0.3)
* For real-world applications, OpenStreetMap data can be used

---

## Conclusion

This project demonstrates:

* Optimal pathfinding in graphs
* Efficient navigation in static environments
* Adaptive planning in dynamic conditions

---
