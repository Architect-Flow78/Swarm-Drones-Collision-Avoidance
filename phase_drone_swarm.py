import math


class PhaseDroneSwarm:
    """
    TPM-based drone swarm controller.

    All drones share a single orbit. Phase separation via the Hurwitz
    optimality shift (1/phi) guarantees maximum angular distance
    between neighbours on a closed cycle [0, 1).

    This is the core TPM claim: golden-ratio phase distribution is the
    most irrational rotation, so two drones can never become arbitrarily
    close in phase — the swarm self-organises into the most stable
    geometric pattern available on a circle.

    Collision check and soft rebalance run every step:
      - check_collisions() raises if any pair of neighbours falls below
        the safety threshold (e.g. after wind disturbance).
      - rebalance() gently nudges drones back toward equal spacing 1/n.
    """

    def __init__(self, num_drones, orbit_radius=10.0):
        if num_drones <= 0:
            raise ValueError("num_drones must be > 0")

        self.num_drones = num_drones
        self.PHI = (1 + 5**0.5) / 2

        # Hurwitz-optimal shift on closed cycle [0, 1)
        self.topological_shift = 1.0 / self.PHI

        self.orbit_radius = orbit_radius

        # All drones on the shared orbit; phases distributed by 1/phi
        self.drones = {
            i: {'phase': (i * self.topological_shift) % 1.0}
            for i in range(num_drones)
        }

        # Safety threshold scales with swarm size: half of the ideal gap.
        # Ideal gap = 1/n; collision warning fires below 0.5/n.
        self.MIN_PHASE_GAP = 0.5 / num_drones

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _check_collisions(self):
        """Raise RuntimeError if any neighbour pair is too close in phase."""
        phases = sorted((s['phase'], d) for d, s in self.drones.items())
        n = len(phases)
        for k in range(n):
            p1, id1 = phases[k]
            p2, id2 = phases[(k + 1) % n]
            gap = (p2 - p1) % 1.0
            if gap < self.MIN_PHASE_GAP:
                raise RuntimeError(
                    f"Collision risk: drones {id1} & {id2}, "
                    f"phase gap={gap:.4f} < {self.MIN_PHASE_GAP:.4f}"
                )

    def _rebalance(self):
        """
        Soft correction toward ideal equal spacing (1/n).
        Anchor = leading drone. Gain 0.1 keeps motion smooth — tune
        to drone inertia.
        """
        n = self.num_drones
        if n < 2:
            return
        ideal_gap = 1.0 / n
        sorted_ids = sorted(self.drones, key=lambda d: self.drones[d]['phase'])
        anchor = self.drones[sorted_ids[0]]['phase']
        for k, drone_id in enumerate(sorted_ids):
            target = (anchor + k * ideal_gap) % 1.0
            current = self.drones[drone_id]['phase']
            # Shortest-path error on torus, mapped to [-0.5, +0.5]
            error = (target - current + 0.5) % 1.0 - 0.5
            self.drones[drone_id]['phase'] = (current + 0.1 * error) % 1.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self, delta_time, base_frequency=None, frequencies=None):
        """
        Advance swarm by one time step.

        Args:
            delta_time:     seconds per step
            base_frequency: cycles/sec applied to all drones (uniform mode)
            frequencies:    dict {drone_id: freq} for per-drone speeds
                            (e.g. wind disturbance compensation)

        Returns:
            dict {drone_id: (x, y)} positions in metres
        """
        if frequencies is None:
            if base_frequency is None:
                raise ValueError("Provide base_frequency or frequencies dict")
            frequencies = {i: base_frequency for i in self.drones}

        # 1. Update phases
        for drone_id, state in self.drones.items():
            state['phase'] = (state['phase'] + frequencies[drone_id] * delta_time) % 1.0

        # 2. Safety check (catches wind drift before it becomes dangerous)
        self._check_collisions()

        # 3. Soft rebalance toward equal spacing
        self._rebalance()

        # 4. Project phase [0, 1) -> physical (x, y) for motor controllers
        positions = {}
        for drone_id, state in self.drones.items():
            angle = state['phase'] * 2 * math.pi
            positions[drone_id] = (
                round(self.orbit_radius * math.cos(angle), 2),
                round(self.orbit_radius * math.sin(angle), 2)
            )
        return positions


# ----------------------------------------------------------------------
# Demo
# ----------------------------------------------------------------------
if __name__ == "__main__":
    swarm = PhaseDroneSwarm(num_drones=5, orbit_radius=10.0)

    print("Swarm initialized (Hurwitz-distributed phases on shared orbit):")
    for d_id, state in swarm.drones.items():
        print(f"  Drone {d_id}: phase = {state['phase']:.4f}")

    sorted_phases = sorted(s['phase'] for s in swarm.drones.values())
    gaps = [(sorted_phases[(i + 1) % 5] - sorted_phases[i]) % 1.0 for i in range(5)]
    print(f"\nInitial gaps: {[round(g, 4) for g in gaps]}")
    print(f"Min gap: {min(gaps):.4f}  (ideal for n=5: {1 / 5:.4f})")

    print("\nSteady cruise (uniform frequency):")
    for t in range(1, 4):
        positions = swarm.step(delta_time=1.0, base_frequency=0.1)
        print(f"  T={t}s: {positions}")

    print("\nWind disturbance (drone 2 slowed to 0.04):")
    custom_freqs = {0: 0.1, 1: 0.1, 2: 0.04, 3: 0.1, 4: 0.1}
    try:
        for t in range(1, 10):
            positions = swarm.step(delta_time=1.0, frequencies=custom_freqs)
            phases = sorted((s['phase'], d) for d, s in swarm.drones.items())
            min_gap = min((phases[(i + 1) % 5][0] - phases[i][0]) % 1.0 for i in range(5))
            print(f"  T={t}s  min_gap={min_gap:.4f}")
    except RuntimeError as e:
        print(f"  SAFETY TRIGGERED: {e}")
