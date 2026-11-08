import random
import math

GridTimelineNetwork_Width = 5
GridTimelineNetwork_Height = 5

class GridWaveSimulator:
    def __init__(self, width=5, height=5, timeline_length=10):
        self.width = width
        self.height = height
        self.length = timeline_length
        self.directions = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
        
        # Initialize grid: each cell has a timeline and a local density (mass) map
        self.grid_timelines = {
            (x, y): [random.choice(self.directions) for _ in range(timeline_length)]
            for x in range(width) for y in range(height)
        }
        self.grid_densities = {
            (x, y): [1.0 for _ in range(timeline_length)]
            for x in range(width) for y in range(height)
        }

    def _axis_to_vector(self, axis):
        mapping = {
            '+X': (1, 0, 0), '-X': (-1, 0, 0),
            '+Y': (0, 1, 0), '-Y': (0, -1, 0),
            '+Z': (0, 0, 1), '-Z': (0, 0, -1)
        }
        return mapping[axis]

    def inject_binary_inspiral_source(self, center_x, center_y, time_step):
        """
        Simulates a high-energy dynamic source (like two masses orbiting) 
        that periodically perturbs local densities and axis states.
        """
        print(f"\n[Gravitational Source] Dynamic inspiral event at grid ({center_x}, {center_y}), t={time_step}")
        
        # Create a transient mass quadrupole fluctuation
        for x in range(self.width):
            for y in range(self.height):
                dist = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                # Quadrupole-like spatial modulation factor
                quadrupole_factor = math.cos(2.0 * dist - float(time_step))
                self.grid_densities[(x, y)][time_step] += abs(quadrupole_factor) * 2.5

    def propagate_gravitational_waves(self, steps=3):
        """
        Simulates how spacetime ripples travel across the grid. 
        Neighboring timelines synchronize their past-future consistency contracts,
        creating traveling wave patterns in directional orientation.
        """
        print(f"\n[Wave Propagation] Simulating {steps} wave cycles across timeline hypergraph...")
        
        for step in range(steps):
            new_timelines = {k: list(v) for k, v in self.grid_timelines.items()}
            
            for x in range(self.width):
                for y in range(self.height):
                    for t in range(1, self.length - 1):
                        # Gather spatial neighbors (von Neumann neighborhood)
                        neighbors = []
                        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height:
                                neighbors.append(self.grid_timelines[(nx, ny)][t])
                        
                        if not neighbors:
                            continue
                            
                        # Local density controls how fast the node yields to neighboring wave pressure
                        local_density = self.grid_densities[(x, y)][t]
                        if random.random() > (1.0 / local_density):
                            continue # Denser regions resist wave distortion (gravitational shielding/delay)
                            
                        # Align node state with the dominant directional vector among spatial neighbors
                        neighbor_vecs = [self._axis_to_vector(n) for n in neighbors]
                        avg_vec = (
                            sum(v[0] for v in neighbor_vecs) / len(neighbor_vecs),
                            sum(v[1] for v in neighbor_vecs) / len(neighbor_vecs),
                            sum(v[2] for v in neighbor_vecs) / len(neighbor_vecs)
                        )
                        
                        # Select closest discrete axis to the spatial average vector
                        best_axis = min(
                            self.directions,
                            key=lambda ax: sum((self._axis_to_vector(ax)[i] - avg_vec[i])**2 for i in range(3))
                        )
                        new_timelines[(x, y)][t] = best_axis
                        
            self.grid_timelines = new_timelines


# --- Execution Demonstration ---
if __name__ == "__main__":
    random.seed(1337)
    
    sim = GridWaveSimulator(width=4, height=4, timeline_length=8)
    
    print("Initial Grid State at Center (2, 2):")
    print("Timelines before wave:", sim.grid_timelines[(2, 2)])
    
    # Step 1: Inject a high-energy source event at t=4
    sim.inject_binary_inspiral_source(center_x=2, center_y=2, time_step=4)
    
    # Step 2: Propagate the resulting gravitational ripples across neighboring spatial nodes
    sim.propagate_gravitational_waves(steps=3)
    
    print("\nGrid State after Wave Propagation:")
    print("Timelines at Center (2, 2)  :", sim.grid_timelines[(2, 2)])
    print("Timelines at Neighbor (2, 3):", sim.grid_timelines[(2, 3)])
    print("Density Profile at Center   :", [round(d, 2) for d in sim.grid_densities[(2, 2)]])
