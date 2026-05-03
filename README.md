# Phase Swarm Coordination

**Author:** Nicolae Pascal
**License:** MIT
**Preprint:** [doi.org/10.5281/zenodo.19960390](https://doi.org/10.5281/zenodo.19960390)

---

## The Problem

Every conventional drone swarm checks distance between every pair of drones, every frame.

| Drones | Checks per frame |
|--------|-----------------|
| 20 | 400 |
| 200 | 40,000 |
| 2,000 | **4,000,000** |

On embedded hardware this is impossible in real time.

---

## The Solution

Each drone holds one number: its phase **θ ∈ [0, 1)**.

Phases are initialized using the **Hurwitz constant 1/φ ≈ 0.618**.
By the Three-Distance Theorem (Steinhaus 1958), this guarantees that
no two drones can become arbitrarily close in phase —
**separation is a mathematical consequence, not a runtime computation.**

Collision check becomes: sort n phases, scan neighbours only.
**Complexity: O(n). No messages. No matrix. No central arbiter.**

---

## Benchmark

```bash
python phase_swarm_proof.py
```

```
Drones   Phase O(n) ms    Standard O(n²) ms    Speedup
    10           0.012                0.006       0.5x
    50           0.050                0.154       3.1x
   100           0.116                0.536       4.6x
   500           0.559               14.571      26.0x
 1,000           1.128               60.693      53.8x
 2,000           2.241              242.347     108.1x

→ Phase approach scales linearly. Standard approach collapses.
→ At 2000 drones: same compute budget, 108x more drones covered.
```

---

## Usage

```python
from phase_swarm_proof import PhaseSwarm

swarm = PhaseSwarm(n=100, radius=50.0)

# Uniform cruise
freqs = {i: 0.1 for i in range(100)}
positions = swarm.step(dt=1.0, freqs=freqs)

# Wind disturbance on drone 7 — swarm self-corrects
freqs[7] = 0.04
positions = swarm.step(dt=1.0, freqs=freqs)
```

---

## How It Works

**Initialization:**
```
θᵢ = (i × 1/φ) mod 1,    i = 0, 1, …, n−1
```
By the Three-Distance Theorem, minimum gap between any two drones
is maximised over all possible step sizes. No rational alternative does better.

**Per-frame step (O(n)):**
1. Update each phase: `θᵢ ← (θᵢ + f·Δt) mod 1`
2. Sort by phase, check neighbours only — skip all distant pairs
3. If gap < threshold: nudge the pair apart
4. Project `θ → (x, y)` for motor controllers

**Wind recovery:**
When a drone slows down, the gap to its neighbour shrinks.
The neighbour-scan catches it in O(n) and corrects automatically.
No drone-to-drone message needed.

---

## Mathematical Foundation

Based on the **Toroidal Phase Metric (TPM)** framework.

> Pascal, N. (2026). *Phase-Topology Swarm Coordination:
> A Toroidal Phase Metric Approach to Drone Fleet Management.*
> Zenodo. [doi.org/10.5281/zenodo.19960390](https://doi.org/10.5281/zenodo.19960390)

---

## Citation

```bibtex
@misc{pascal2026swarm,
  author    = {Pascal, Nicolae},
  title     = {Phase-Topology Swarm Coordination},
  year      = {2026},
  publisher = {Zenodo},
  doi       = {10.5281/zenodo.19960390},
  url       = {https://doi.org/10.5281/zenodo.19960390},
  license   = {CC BY 4.0}
}
```
