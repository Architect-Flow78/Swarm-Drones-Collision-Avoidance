# Phase Drone Swarm — Toroidal Phase Metric

**Author:** Nicolae Pascal
**License:** MIT
**Preprint:** [doi.org/10.5281/zenodo.19960390](https://doi.org/10.5281/zenodo.19960390)

---

## The Problem

Standard swarm collision avoidance scales as **O(n²)**:
every drone checks distance to every other drone, every frame.

| Fleet size | Checks per frame |
|-----------|-----------------|
| 20 drones | 400 |
| 200 drones | 40 000 |
| 2 000 drones | 4 000 000 |

At scale this floods the CPU, saturates inter-drone messaging,
and makes real-time coordination impossible on embedded hardware.

---

## The Solution

Each drone carries **one scalar** — its phase θ ∈ [0, 1) on a unit torus.

Phases are initialized using the **Hurwitz constant 1/φ ≈ 0.618**
(the golden ratio reciprocal). By the three-distance theorem, this is
the most irrational rotation: no rational step size gives better
worst-case separation. Collision avoidance is therefore guaranteed
by number theory — not by distance calculation.

**Complexity: O(n).** One scalar update per drone per frame.

---

## Benchmark

Run it yourself:

```bash
python phase_drone_swarm_v2.py
```

Results on standard hardware:

```
Drones     TPM O(n) µs    Naive O(n²) µs    Speedup
      10            22              20         0.9x
      50            99             393         4.0x
     100           211           1 669         7.9x
     500         1 096          42 287        38.6x
    1 000         2 088         171 813        82.3x
    2 000         4 343         688 562       158.5x
```

---

## Features

- **Zero inter-drone communication** — separation guaranteed by math, not messaging
- **Wind disturbance recovery** — automatic soft rebalance using shortest-path toroidal correction
- **Drone failure handling** — `remove_drone()` redistributes phases without restarting the swarm
- **Hot join** — `add_drone()` places new drone at the largest gap automatically
- **Health metrics** — `health()` returns uniformity, min/max gap, coverage in degrees
- **Non-crashing collision detection** — warning + emergency rebalance instead of system crash
- **Configurable gain** — tune rebalance speed to drone inertia
- **Runs on edge hardware** — no cloud, no GPS required, pure arithmetic

---

## Quick Start

```python
from phase_drone_swarm_v2 import PhaseDroneSwarm

# Create swarm
swarm = PhaseDroneSwarm(num_drones=10, orbit_radius=15.0)

# Advance one time step — returns {drone_id: (x, y)} in metres
positions = swarm.step(delta_time=1.0, base_frequency=0.1)

# Check swarm health
print(swarm.health())
# {'num_drones': 10, 'min_gap': 0.09, 'uniformity': 0.9, 'coverage_deg': 32.4, ...}

# Simulate wind disturbance on drone 3
freqs = {i: 0.1 for i in range(10)}
freqs[3] = 0.04   # drone 3 slowed to 40%
positions = swarm.step(delta_time=1.0, frequencies=freqs)
# Rebalancer corrects automatically — no crash, no manual intervention

# Handle drone failure
swarm.remove_drone(3)   # swarm self-reorganises to 9 drones

# Add replacement drone
swarm.add_drone(10)     # placed at largest gap automatically
```

---

## How It Works

### Phase initialization

```
θᵢ = (i × 1/φ) mod 1,    i = 0, 1, …, n−1
```

By the **three-distance theorem** (Steinhaus 1958), this partitions
[0, 1) into gaps of at most three distinct lengths, with the minimum
gap maximised over all possible step sizes.

### Phase dynamics

```
θᵢ(t + Δt) = (θᵢ(t) + f × Δt) mod 1
```

Under uniform frequency f, relative phases are invariant —
Hurwitz separation is preserved indefinitely without any correction.

### Toroidal rebalance (wind recovery)

After disturbance, each follower drone k is nudged toward its target:

```
target  = (θ_anchor + k/n) mod 1
error   = (target − θₖ + 0.5) mod 1 − 0.5   ← shortest path on torus
θₖ     ← θₖ + α × error                      ← α = rebalance gain
```

The `+0.5 mod 1 − 0.5` maps the error to [−0.5, +0.5], ensuring
the drone always moves along the **shorter arc** — never making a
full revolution to close a small gap.

### Physical projection

```
x = r × cos(2π θ),    y = r × sin(2π θ)
```

Motor controllers receive only (x, y) pairs.
All coordination logic stays in phase space.

---

## Files

| File | Description |
|------|-------------|
| `phase_drone_swarm_v2.py` | Main controller + benchmark + demo |
| `LICENSE` | MIT License |

---

## Mathematical Foundation

Based on the **Toroidal Phase Metric (TPM)** framework.
Full theoretical treatment in the Zenodo preprint:

> Pascal, N. (2026). *Phase-Topology Swarm Coordination:
> A Toroidal Phase Metric Approach to Drone Fleet Management.*
> Zenodo. [doi.org/10.5281/zenodo.19960390](https://doi.org/10.5281/zenodo.19960390)

---

## Citation

```bibtex
@misc{pascal2026swarm,
  author    = {Pascal, Nicolae},
  title     = {Phase-Topology Swarm Coordination: A Toroidal Phase Metric
               Approach to Drone Fleet Management},
  year      = {2026},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.19960390},
  url       = {https://doi.org/10.5281/zenodo.19960390},
  license   = {CC BY 4.0}
}
```
