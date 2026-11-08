import random

class CoupledParticleNetwork:
    def __init__(self, length=10, initial_axis='X'):
        self.directions = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
        self.length = length
        
        # Define anti-correlated initial pair for the entanglement origin (t=0)
        axis_pairs = {
            'X': ('+X', '-X'),
            'Y': ('+Y', '-Y'),
            'Z': ('+Z', '-Z')
        }
        init_a, init_b = axis_pairs[initial_axis]
        
        # Initialize timelines sharing an inverted origin state
        self.timeline_a = [init_a] + [random.choice(self.directions) for _ in range(length - 1)]
        self.timeline_b = [init_b] + [random.choice(self.directions) for _ in range(length - 1)]
        
    def _axis_to_vector(self, axis):
        mapping = {
            '+X': (1, 0, 0), '-X': (-1, 0, 0),
            '+Y': (0, 1, 0), '-Y': (0, -1, 0),
            '+Z': (0, 0, 1), '-Z': (0, 0, -1)
        }
        return mapping[axis]

    def _invert_axis(self, axis):
        sign, dim = axis[0], axis[1]
        new_sign = '-' if sign == '+' else '+'
        return f"{new_sign}{dim}"

    def compute_joint_flux(self):
        """Measures total vector sum across both coupled trajectories."""
        vecs_a = [self._axis_to_vector(a) for a in self.timeline_a]
        vecs_b = [self._axis_to_vector(b) for b in self.timeline_b]
        
        sum_a = tuple(sum(v[i] for v in vecs_a) for i in range(3))
        sum_b = tuple(sum(v[i] for v in vecs_b) for i in range(3))
        
        joint_flux = tuple(sum_a[i] + sum_b[i] for i in range(3))
        return sum_a, sum_b, joint_flux

    def measure_particle_a(self, measure_index, observed_axis):
        """
        Simulates measurement collapse on Particle A.
        Enforces state at measure_index, propagates shockwave back to origin (t=0),
        and updates Particle B's history to satisfy the joint consistency contract.
        """
        print(f"\n[Measurement Event] Particle A measured at t={measure_index}: Collapsed to {observed_axis}")
        self.timeline_a[measure_index] = observed_axis
        
        # Retrocausal shockwave: propagate backward from measurement index to t=0 on Particle A
        for t in range(measure_index - 1, -1, -1):
            # Align past nodes toward compatibility with measured outcome
            self.timeline_a[t] = self._resolve_node_contract(self.timeline_a[t], self.timeline_a[t+1])
            
        # Entangled Origin Reaction: Update Particle B's t=0 state to match inverted origin node
        origin_a = self.timeline_a[0]
        self.timeline_b[0] = self._invert_axis(origin_a)
        print(f"  -> Retrocausal contract reached t=0. Particle B origin re-fused to: {self.timeline_b[0]}")
        
        # Procausal Cascade: Update Particle B's future timeline to absorb the boundary change
        for t in range(1, self.length):
            self.timeline_b[t] = self._resolve_node_contract(self.timeline_b[t], self.timeline_b[t-1])

    def _resolve_node_contract(self, current_axis, reference_axis):
        """Helper to contract adjacent nodes into alignment with reference axis."""
        ref_vec = self._axis_to_vector(reference_axis)
        best_axis = min(
            self.directions,
            key=lambda ax: sum((self._axis_to_vector(ax)[i] - ref_vec[i])**2 for i in range(3))
        )
        return best_axis


# --- Simulation Run ---
if __name__ == "__main__":
    random.seed(101)
    
    # Initialize coupled network with an initial X-axis entanglement
    net = CoupledParticleNetwork(length=8, initial_axis='X')
    
    print("Initial Timelines (Pre-Measurement):")
    print("Particle A:", net.timeline_a)
    print("Particle B:", net.timeline_b)
    sum_a, sum_b, joint = net.compute_joint_flux()
    print("Joint System Flux:", joint)
    
    # Measure Particle A at t=5 and force it to collapse to '+Z'
    net.measure_particle_a(measure_index=5, observed_axis='+Z')
    
    print("\nPost-Measurement Timeline (After Past/Future Contract Resolution):")
    print("Particle A:", net.timeline_a)
    print("Particle B:", net.timeline_b)
    sum_a, sum_b, joint = net.compute_joint_flux()
    print("Final System Flux:", joint)
