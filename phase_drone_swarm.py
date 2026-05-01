import math


class PhaseDroneSwarm:
    def __init__(self, num_drones):
        if num_drones <= 0:
            raise ValueError("num_drones must be > 0")

        self.num_drones = num_drones
        self.PHI = (1 + 5**0.5) / 2

        # [AXIOM]: Absolute unit of closed cycle = 1
        # Hurwitz-optimal shift guarantees maximum phase separation on [0, 1)
        self.topological_shift = 1.0 / self.PHI

        # Each drone gets a unique phase and orbit radius
        self.drones = {
            i: {
                'phase': (i * self.topological_shift) % 1.0,
                'radius': (i + 1) * 2.5  # orbit spacing in meters
            }
            for i in range(num_drones)
        }

        # Group drone IDs by radius for collision checks and rebalancing
        self._orbits = {}
        for drone_id, state in self.drones.items():
            r = state['radius']
            self._orbits.setdefault(r, []).append(drone_id)

        self.MIN_PHASE_GAP = 0.05  # 5% of torus ~ 18 degrees

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _rebalance_orbit(self, ids):
        """
        Soft phase correction after wind disturbance or frequency drift.
        Anchor = leading drone (lowest phase). Others are gently pushed
        toward ideal equal spacing (1/n).
        Correction gain 0.1 keeps motion smooth; tune to drone inertia.
        """
        n = len(ids)
        if n < 2:
            return

        ideal_gap = 1.0 / n
        sorted_ids = sorted(ids, key=lambda d: self.drones[d]['phase'])
        anchor = self.drones[sorted_ids[0]]['phase']

        for k, drone_id in enumerate(sorted_ids):
            target = (anchor + k * ideal_gap) % 1.0
            current = self.drones[drone_id]['phase']
            # Shortest-path error on torus: maps to [-0.5, +0.5]
            error = (target - current + 0.5) % 1.0 - 0.5
            self.drones[drone_id]['phase'] = (current + 0.1 * error) % 1.0

    def _check_collisions(self):
        """
        Raises RuntimeError if any two drones on the same orbit
        are closer than MIN_PHASE_GAP.
        Only relevant when drones share a radius (custom configurations).
        """
        for r, ids in self._orbits.items():
            if len(ids) < 2:
                continue
            phases = sorted((self.drones[d]['phase'], d) for d in ids)
            n = len(phases)
            for k in range(n):
                p1 = phases[k][0]
                p2 = phases[(k + 1) % n][0]
                gap = (p2 - p1) % 1.0
                if gap < self.MIN_PHASE_GAP:
                    raise RuntimeError(
                        f"Collision risk on radius {r}m: "
                        f"drones {phases[k][1]} & {phases[(k+1)%n][1]}, "
                        f"phase gap={gap:.4f} < {self.MIN_PHASE_GAP}"
                    )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self, delta_time, base_frequency=None, frequencies=None):
        """
        Advance swarm by one time step.

        Args:
            delta_time:     seconds per step
            base_frequency: cycles/sec applied to all drones (uniform mode)
            frequencies:    dict {drone_id: freq} for individual speeds
                            (e.g. after wind disturbance compensation)

        Returns:
            dict {drone_id: (x, y)} physical positions in meters

        Note:
            Phase-separation guarantee holds only when all drones share
            the same radius. With shared radii and different frequencies,
            collision check + rebalance become mandatory.
        """
        if frequencies is None:
            if base_frequency is None:
                raise ValueError("Provide base_frequency or frequencies dict")
            frequencies = {i: base_frequency for i in self.drones}

        # 1. Update phases (TPM topology)
        for drone_id, state in self.drones.items():
            freq = frequencies[drone_id]
            state['phase'] = (state['phase'] + freq * delta_time) % 1.0

        # 2. Collision check (catches wind drift before it becomes dangerous)
        self._check_collisions()

        # 3. Soft rebalance — gently correct spacing on each orbit
        for ids in self._orbits.values():
            self._rebalance_orbit(ids)

        # 4. Project phase [0, 1) -> physical (x, y) for motor controllers
        physical_positions = {}
        for drone_id, state in self.drones.items():
            angle = state['phase'] * 2 * math.pi
            physical_positions[drone_id] = (
                round(state['radius'] * math.cos(angle), 2),
                round(state['radius'] * math.sin(angle), 2)
            )

        return physical_positions


# ----------------------------------------------------------------------
# Test run
# ----------------------------------------------------------------------
if __name__ == "__main__":
    swarm = PhaseDroneSwarm(num_drones=5)

    print("Swarm initialized (phase coordinates [0, 1)):")
    for d_id, state in swarm.drones.items():
        print(f"  Drone {d_id}: phase = {state['phase']:.4f}, "
              f"radius = {state['radius']}m")

    print("\nPatrol simulation (physical X, Y in meters):")
    for t in range(1, 4):
        positions = swarm.step(delta_time=1.0, base_frequency=0.1)
        print(f"  T={t}s: {positions}")

    print("\nWind disturbance test (drone 2 slows down):")
    custom_freqs = {0: 0.1, 1: 0.1, 2: 0.04, 3: 0.1, 4: 0.1}
    for t in range(1, 4):
        positions = swarm.step(delta_time=1.0, frequencies=custom_freqs)
        print(f"  T={t}s: {positions}")
