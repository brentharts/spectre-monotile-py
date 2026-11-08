import random

class GravitationalTimelineNetwork:
    def __init__(self, length=10, initial_axis='X'):
        self.directions = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
        self.length = length
        
        # Define baseline timeline and local node density (mass/energy distribution)
        # Default density is 1.0; higher values create "gravitational wells"
        self.timeline = [random.choice(self.directions) for _ in range(length)]
        self.density_profile = [1.0 for _ in range(length)]
        
    def _axis_to_vector(self, axis):
        mapping = {
            '+X': (1, 0, 0), '-X': (-1, 0, 0),
            '+Y': (0, 1, 0), '-Y': (0, -1, 0),
            '+Z': (0, 0, 1), '-Z': (0, 0, -1)
        }
        return mapping[axis]

    def set_gravitational_well(self, center_index, weight=3.5):
        """
        Creates a high-density mass concentration at a specific timeline index,
        simulating a heavy particle or black hole core.
        """
        print(f"\n[Mass Injection] Introducing gravitational well at t={center_index} with density factor {weight}")
        for t in range(self.length):
            # Lorentzian or Gaussian-like drop-off around the center
            distance = abs(t - center_index)
            self.density_profile[t] += max(0.0, weight / (1.0 + distance**2))

    def compute_local_clock_rate(self, t):
        """
        Time dilation factor: Higher density leads to lower effective clock rate 
        (more microscopic axis interactions per macroscopic step).
        """
        return 1.0 / self.density_profile[t]

    def apply_gravitational_relaxation(self, shock_index, new_axis):
        """
        Injects an event and solves the consistency contract, taking into account 
        that high-density regions resist reordering (stretching past timelines).
        """
        print(f"\n[Event Trigger] State change at t={shock_index} forced to {new_axis}")
        self.timeline[shock_index] = new_axis
        
        max_iterations = 25
        for iteration in range(max_iterations):
            changed = False
            # Ripple outward, weighted by local density resistance
            for t in range(1, self.length - 1):
                # High density regions dampen rapid fluctuations (time dilation inertia)
                resistance = self.density_profile[t]
                if random.random() > (1.0 / resistance):
                    continue  # Denser nodes skip rapid updates, holding past states longer
                
                # Align node with neighbors weighted by clock rate
                ref_vec = self._axis_to_vector(self.timeline[t-1])
                best_axis = min(
                    self.directions,
                    key=lambda ax: sum((self._axis_to_vector(ax)[i] - ref_vec[i])**2 for i in range(3))
                )
                if self.timeline[t] != best_axis:
                    self.timeline[t] = best_axis
                    changed = True
            
            if not changed:
                break

        print(f"  -> Contract settled after gravitational dampening.")


# --- Execution Demonstration ---
if __name__ == "__main__":
    random.seed(42)
    
    net = GravitationalTimelineNetwork(length=10)
    print("Baseline Timeline:")
    print("Axes   :", net.timeline)
    print("Density:", [round(d, 2) for d in net.density_profile])
    
    # Place a heavy mass concentration around t=6 (simulating a dense gravitational center)
    net.set_gravitational_well(center_index=6, weight=4.0)
    
    print("\nUpdated Density Profile (Time Dilation Map):")
    print("Clock Rates (1/density):", [round(net.compute_local_clock_rate(t), 2) for t in range(net.length)])
    
    # Trigger a high-energy perturbation
    net.apply_gravitational_relaxation(shock_index=2, new_axis='+Z')
    
    print("\nFinal Timeline after Gravitational Contract Resolution:")
    print("Axes   :", net.timeline)
