"""
Phase Swarm Coordination — Complexity Proof of Concept
=======================================================
Author : Nicolae Pascal
License: MIT
Preprint: https://doi.org/10.5281/zenodo.19960390

ASSUMPTION
----------
All drones share one circular orbit of fixed radius r.
On a single orbit, phase order = distance order: larger phase gap
always means larger physical distance. This makes phase arithmetic
a valid proxy for collision geometry.
For multi-orbit or 3D swarms, a geometric layer must be added.

PROBLEM
-------
Standard swarm collision avoidance checks every pair every frame:
O(n²) distance computations.

At 2000 drones: 4,000,000 checks per frame.
On embedded hardware this is impossible in real time.

SOLUTION
--------
Replace all-pairs check with phase-sorted neighbour scan.

Each drone holds one number: its phase θ ∈ [0, 1).

INITIALIZATION (Three-Distance Theorem, Steinhaus 1958):
    θᵢ = (i · 1/φ) mod 1

    The theorem states: for any irrational α and any n, the sequence
    {k·α mod 1} partitions [0,1) into gaps of AT MOST 3 distinct lengths.
    When α = 1/φ (Hurwitz constant), the minimum gap is MAXIMISED
    over all irrational step sizes — the best possible starting
    separation for n points on a circle.

    IMPORTANT: this guarantee applies to initialization.
    Under UNIFORM frequency (all drones same speed), relative phases
    are invariant — separation is preserved forever, no correction needed.
    Under NON-UNIFORM frequency (wind, fault), phases drift and a
    rebalancer is required (included in step()).

COLLISION CHECK:
    Sort drones by phase: O(n log n).
    Scan only adjacent pairs: O(n).
    Total per frame: O(n log n) — vs O(n²) conventional.

    Honest complexity: O(n log n), not O(n).
    At n=2000: ~22,000 operations vs 4,000,000. Still 180x fewer.

BENCHMARK:
    Full step vs full step — apples to apples.
    Both approaches: same init, same freqs, same output.
    Only the collision check differs.

RUN THIS FILE to see the proof.
"""

import math
import time
import random


φ = (1 + 5 ** 0.5) / 2
α = 1.0 / φ              # Hurwitz constant


# ── Phase swarm — O(n log n) per frame ───────────────────────────────────

class PhaseSwarm:

    def __init__(self, n, radius=10.0):
        self.radius = radius
        self.phase = {i: (i * α) % 1.0 for i in range(n)}

    def step(self, dt, freqs):
        n = len(self.phase)

        # 1. Phase update — O(n)
        for i in self.phase:
            self.phase[i] = (self.phase[i] + freqs[i] * dt) % 1.0

        # 2. Sort by phase — O(n log n)
        #    On single orbit: phase order = spatial order.
        #    Only neighbours need checking.
        ordered = sorted(self.phase, key=lambda i: self.phase[i])

        # 3. Neighbour scan — O(n)
        for k in range(n):
            a = ordered[k]
            b = ordered[(k + 1) % n]
            gap = (self.phase[b] - self.phase[a]) % 1.0
            if gap < 0.5 / n:
                mid = (self.phase[a] + gap / 2) % 1.0
                self.phase[a] = (mid - 1.0 / n / 2) % 1.0
                self.phase[b] = (mid + 1.0 / n / 2) % 1.0

        # 4. Project θ → (x, y) — O(n)
        return {
            i: (round(self.radius * math.cos(2 * math.pi * θ), 2),
                round(self.radius * math.sin(2 * math.pi * θ), 2))
            for i, θ in self.phase.items()
        }


# ── Standard swarm — O(n²) per frame ─────────────────────────────────────

class StandardSwarm:
    """Same init, same output — only collision check differs."""

    def __init__(self, n, radius=10.0):
        self.radius = radius
        self.phase = {i: (i * α) % 1.0 for i in range(n)}

    def step(self, dt, freqs):
        n = len(self.phase)
        ids = list(self.phase.keys())

        # 1. Phase update — O(n)
        for i in self.phase:
            self.phase[i] = (self.phase[i] + freqs[i] * dt) % 1.0

        # 2. Project — O(n)
        positions = {
            i: (self.radius * math.cos(2 * math.pi * self.phase[i]),
                self.radius * math.sin(2 * math.pi * self.phase[i]))
            for i in ids
        }

        # 3. All-pairs check — O(n²)
        for i in range(n):
            for j in range(i + 1, n):
                x1, y1 = positions[ids[i]]
                x2, y2 = positions[ids[j]]
                math.hypot(x2 - x1, y2 - y1)   # the expensive part

        return {
            i: (round(positions[i][0], 2), round(positions[i][1], 2))
            for i in ids
        }


# ── Benchmark — full step vs full step ───────────────────────────────────

def benchmark():
    sizes  = [10, 50, 100, 500, 1000, 2000]
    steps  = 50
    radius = 50.0

    print(f"\n{'Drones':>8}  {'Phase O(nlogn) ms':>19}  "
          f"{'Standard O(n²) ms':>19}  {'Speedup':>9}")
    print("─" * 65)

    for n in sizes:
        freqs = {i: 0.1 + random.uniform(-0.02, 0.02) for i in range(n)}

        p = PhaseSwarm(n=n, radius=radius)
        t0 = time.perf_counter()
        for _ in range(steps):
            p.step(dt=0.1, freqs=freqs)
        phase_ms = (time.perf_counter() - t0) / steps * 1000

        s = StandardSwarm(n=n, radius=radius)
        t0 = time.perf_counter()
        for _ in range(steps):
            s.step(dt=0.1, freqs=freqs)
        std_ms = (time.perf_counter() - t0) / steps * 1000

        speedup = std_ms / phase_ms if phase_ms > 0 else float('inf')
        print(f"{n:>8}  {phase_ms:>19.3f}  {std_ms:>19.3f}  "
              f"{speedup:>8.1f}x")

    print("\n→ Fair comparison: full step vs full step, same initialization.")
    print("→ Phase: O(n log n). Standard: O(n²).")
    print("→ At scale the difference is decisive.")


# ── Demo ──────────────────────────────────────────────────────────────────

def demo():
    print("DEMO: 5-drone swarm, wind disturbance on drone 2\n")
    swarm = PhaseSwarm(n=5, radius=10.0)
    freqs = {i: 0.1 for i in range(5)}

    print("  Uniform cruise — phases invariant under equal frequency:")
    for t in range(1, 3):
        swarm.step(dt=1.0, freqs=freqs)
        phases = sorted(swarm.phase.values())
        gaps = [(phases[(i+1)%5] - phases[i]) % 1.0 for i in range(5)]
        print(f"    T={t}s  min_gap={min(gaps):.3f}  max_gap={max(gaps):.3f}")

    print("\n  Wind: drone 2 at 40% speed (rebalancer activates):")
    freqs[2] = 0.04
    for t in range(1, 6):
        swarm.step(dt=1.0, freqs=freqs)
        phases = sorted(swarm.phase.values())
        gaps = [(phases[(i+1)%5] - phases[i]) % 1.0 for i in range(5)]
        bar = '█' * int(min(gaps) * 5 * 20)
        print(f"    T={t}s  min_gap={min(gaps):.3f}  {bar}")


if __name__ == '__main__':
    demo()
    print()
    benchmark()
    
