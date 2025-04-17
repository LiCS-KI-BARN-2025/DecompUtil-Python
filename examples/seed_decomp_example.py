#!/usr/bin/env python3
import numpy as np
from pydecomp_util import *

def main():
    print("2D Seed Decomp Example:")
    
    try:
        # Create obstacles directly (matching test_seed_decomp.cpp)
        obs = np.array([
            [-0.2, 1.5],
            [1.0, 0.0],
            [0.8, -1.0],
            [-0.5, -0.5]
        ], dtype=np.float64)
        print(f"Created {len(obs)} obstacles")

        # Create seed point
        seed = np.array([0.0, 0.0], dtype=np.float64)
        print(f"Creating seed point at: {seed}")
        
        # Create and initialize SeedDecomp2D
        decomp = SeedDecomp2D()  # Create without args first
        decomp.set_obs(obs)  # Set obstacles before other operations
        print("Set obstacles")
        
        bbox = np.array([2.0, 2.0], dtype=np.float64)
        print(f"Setting bbox: {bbox}")
        decomp.set_local_bbox(bbox)
        print("Local bbox set")
        
        print("Dilating...")
        decomp.dilate(0.1)  # Use same radius as C++ test
        print("Dilation complete")
        
        # Verify seed position
        seed_pos = decomp.get_seed()
        print(f"Seed position: {seed_pos}")
        
        # Visualization with matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        
        plt.figure(figsize=(10, 10))
        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        
        # Plot obstacles as red dots
        plt.scatter(obs[:, 0], obs[:, 1], c='red', s=50, label='Obstacles')
        
        # Plot seed point as a star
        plt.scatter([seed_pos[0]], [seed_pos[1]], c='red', marker='*', s=200, label='Seed Point')
        
        # Plot ellipsoid
        ellipsoid = decomp.get_ellipsoid()
        ellipse_pts = np.array(ellipsoid.sample(100))
        plt.plot(ellipse_pts[:, 0], ellipse_pts[:, 1], 'c-', alpha=0.4, linewidth=2, label='Ellipsoid')
        
        # Plot polyhedron
        poly = decomp.get_polyhedron()
        vertices = poly.vertices()
        if len(vertices) > 0:
            poly_patch = Polygon(vertices, alpha=0.2, color='blue', label='Polyhedron')
            plt.gca().add_patch(poly_patch)
        
        plt.grid(True)
        plt.legend()
        plt.title('Seed Decomposition')
        plt.axis('equal')
        plt.show()
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()