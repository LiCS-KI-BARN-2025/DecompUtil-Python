#!/usr/bin/env python3
import numpy as np
from pydecomp_util import *

def main():
    print("2D Iterative Decomp Example:")
    
    try:
        # Create obstacles directly
        obs = np.array([
            [-0.2, 1.5],
            [1.0, 0.0],
            [0.8, -1.0],
            [-0.5, -0.5]
        ], dtype=np.float64)
        print(f"Created {len(obs)} obstacles")

        # Create path
        path = np.array([
            [-1.5, 0.0],
            [1.5, 0.3]
        ], dtype=np.float64)
        
        print("Creating IterativeDecomp2D...")
        decomp = IterativeDecomp2D()  # Create without args first
        decomp.set_obs(obs)  # Set obstacles before other operations
        print("Set obstacles")
        
        bbox = np.array([2.0, 2.0], dtype=np.float64)
        print(f"Setting bbox: {bbox}")
        decomp.set_local_bbox(bbox)
        print("Local bbox set")
        
        print("Dilating path iteratively...")
        decomp.dilate_iter(path, 5, 0.3, 0.0)
        print("Iterative dilation complete")
        
        # Visualization with matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        
        plt.figure(figsize=(10, 10))
        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        
        # Plot obstacles as red dots
        plt.scatter(obs[:, 0], obs[:, 1], c='red', s=50, label='Obstacles')
        
        # Plot path as red line
        plt.plot(path[:, 0], path[:, 1], 'r-', linewidth=2, label='Path')
        
        # Plot ellipsoids
        for ellipsoid in decomp.get_ellipsoids():
            ellipse_pts = np.array(ellipsoid.sample(100))
            plt.plot(ellipse_pts[:, 0], ellipse_pts[:, 1], 'c-', alpha=0.4, linewidth=2)
        plt.plot([], [], 'c-', alpha=0.4, linewidth=2, label='Ellipsoids')  # For legend
        
        # Plot polyhedrons
        for polyhedron in decomp.get_polyhedrons():
            vertices = polyhedron.vertices()
            if len(vertices) > 0:
                poly_patch = Polygon(vertices, alpha=0.2, color='blue')
                plt.gca().add_patch(poly_patch)
        plt.plot([], [], 'b-', alpha=0.2, label='Polyhedrons')  # For legend
        
        plt.grid(True)
        plt.legend()
        plt.title('Iterative Decomposition')
        plt.axis('equal')
        plt.show()
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()