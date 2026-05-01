# Swarm-Drones-Collision-Avoidance
# Swarm Drones: Collision Avoidance via Toroidal Phase Metric (TPM)

**Author:** Nicolae Pascal / Independent Researcher  
**Status:** Proof of Concept (Python Simulation)

## Abstract
Current swarm robotics relies on a Cartesian spatial paradigm. To avoid collisions, algorithms must continuously scan physical coordinates (X, Y, Z), calculate velocity vectors, and predict spatial intersections using resource-intensive matrix mathematics. This repository introduces a fundamentally different approach: replacing spatial coordinate tracking with pure Phase Topology. By applying the Toroidal Phase Metric (TPM), we guarantee collision avoidance not through heavy computation, but through the fundamental geometry of the phase space.

## 1. Theoretical Foundation: The Phase Paradigm

In standard coordinate systems, the basic unit of measurement is a linear segment or a cube. In TPM, the absolute unit of computation is a closed topological cycle (a circle or sphere).

### Core Axioms of the Algorithm:
1. **The Absolute Cycle:** The absolute unit of a closed topological cycle is strictly defined as $\pi = 1$. This eliminates the infinite irrational tails of standard trigonometry, allowing for perfect computation of cyclic states using simple algebra (modulo operations).
2. **Topological Shielding:** Instead of radar proximity alerts, the initial phase state of each drone is distributed using a Hurwitz-optimal partition. By applying a topological shift based on the Golden Ratio operator ($\Phi = 1.618...$), the system intrinsically forms a state where agents mathematically cannot occupy the same phase at the same time.

## 2. Advantages over Standard Models

* **Zero Cartesian Collision Checks:** The algorithm does not compute distances between drones. Safety is maintained purely by managing phase values.
* **Extreme Computational Efficiency:** Replaces heavy proximity matrix calculations with simple scalar phase addition, executable on the most basic microcontrollers.
* **Inherent Fault Tolerance (Wind Compensation):** The system features a "soft rebalance" mechanism. If a drone slows down due to external disturbances (e.g., wind, motor degradation), the algorithm automatically adjusts the topological gaps (`_rebalance_orbit`). The swarm acts as a "phase spring," maintaining optimal spacing without reactive spatial commands.

## 3. Repository Contents
* `swarm_tpm.py` - The core Python class `PhaseDroneSwarm` and the Matplotlib visualization demonstrating wind disturbance compensation.

## 4. Quick Start (Simulation)

### Requirements
The code is written in pure Python. The visualization requires the `matplotlib` library.
```bash
pip install matplotlib
Running the Code
Run the script from your terminal to view the real-time simulation:

Bash
python swarm_tpm.py
Understanding the Visualization
When you run the script, you will see a 2D projection of the phase torus:

Frames 0-50: The swarm initializes and uniformly distributes its phase states to achieve maximum separation.
Frames 50-150 (Disturbance Injection): Drone №2 simulates a hardware fault or wind resistance and drastically reduces its frequency. You will observe the topological rebalancing in real-time: the other drones will fluidly adjust their phase gaps to prevent collision, acting as a unified topological structure.
5. Collaboration and Licensing
This architecture represents a shift from spatial robotics to topological phase control. The concept and code are open for study and collaboration. I welcome discussions with engineers and developers in the fields of autonomous systems, UAVs, and satellite constellations.
