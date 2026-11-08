import random

class AxisNetwork:
    def __init__(self, length=12):
        # The 6 fundamental directional axes
        self.directions = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
        # A trajectory is a chronological list of axis interaction nodes
        self.timeline = [random.choice(self.directions) for _ in range(length)]
        
    def _axis_to_vector(self, axis):
        mapping = {
            '+X': (1, 0, 0), '-X': (-1, 0, 0),
            '+Y': (0, 1, 0), '-Y': (0, -1, 0),
            '+Z': (0, 0, 1), '-Z': (0, 0, -1)
        }
        return mapping[axis]

    def compute_net_flux(self):
        """Calculates total vector sum representing the global energy/momentum background."""
        vecs = [self._axis_to_vector(a) for a in self.timeline]
        return tuple(sum(v[i] for v in vecs) for i in range(3))

    def inject_shockwave(self, index, new_axis):
        """
        Injects a high-energy event at time index 'index', initiating 
        a retrocausal and procausal relaxation contract across the timeline.
        """
        print(f"\n[Shockwave] High-energy event at t={index}: forced axis change to {new_axis}")
        self.timeline[index] = new_axis
        
        # Run bidirectional consistency contract solver
        self._resolve_consistency(index)

    def _resolve_consistency(self, shock_index):
        """
        Iterative relaxation contract ensuring past and future nodes 
        mutually adjust to satisfy global conservation constraints.
        """
        max_iterations = 30
        for iteration in range(max_iterations):
            flux = self.compute_net_flux()
            flux_magnitude = sum(x**2 for x in flux)
            
            # Target equilibrium threshold
            if flux_magnitude <= 3:
                print(f"  -> Contract converged at iteration {iteration}. Balanced flux: {flux}")
                return
            
            # Ripple outwards from the shock index in both past and future directions
            spread = (iteration // 3) + 1
            for offset in range(-spread, spread + 1):
                target = shock_index + offset
                if 0 <= target < len(self.timeline) and target != shock_index:
                    current_vec = self._axis_to_vector(self.timeline[target])
                    # Shift adjacent node toward a direction that neutralizes excess flux
                    best_axis = min(
                        self.directions, 
                        key=lambda ax: sum((current_vec[i] + self._axis_to_vector(ax)[i] + flux[i])**2 for i in range(3))
                    )
                    self.timeline[target] = best_axis

        print("  -> Notice: Contract reached max iterations. Partial consistency achieved.")

# --- Execution Demonstration ---
if __name__ == "__main__":
    random.seed(42)
    net = AxisNetwork(length=12)
    
    print("Initial Axis Timeline:")
    print(net.timeline)
    print("Initial Net Flux:", net.compute_net_flux())
    
    # Inject a high-energy shockwave right in the middle of the timeline
    net.inject_shockwave(index=6, new_axis='-X')
    
    print("\nRelaxed Timeline after Consistency Contract:")
    print(net.timeline)
    print("Final Net Flux:", net.compute_net_flux())
