#!/usr/bin/env python3
import numpy as np
from pydecomp_util import *

def read_obstacles(filename):
    """Read obstacles from file"""
    obstacles = []
    with open(filename, 'r') as f:
        for line in f:
            coords = [float(x) for x in line.strip().split()]
            obstacles.append(coords)
    return np.array(obstacles, dtype=np.float64)

def main():
    print("2D Example:")
    
    try:
        # Read obstacles from the same file as the C++ test
        obs = read_obstacles("data/obstacles.txt")
        print(f"Loaded {len(obs)} obstacles")

        # Create line segment with exact same coordinates as test
        pos1 = np.array([-1.5, 0.0], dtype=np.float64)
        pos2 = np.array([1.5, 0.3], dtype=np.float64)
        
        print(f"Creating LineSegment2D with endpoints: {pos1} -> {pos2}")
        line = LineSegment2D(pos1, pos2)
        print("LineSegment2D created with endpoints")
        
        print("Setting obstacles...")
        line.set_obs(obs)
        print(f"Set {len(obs)} obstacles")
        
        bbox = np.array([2.0, 2.0], dtype=np.float64)
        print(f"Setting bbox: {bbox}")
        line.set_local_bbox(bbox)
        print("Local bbox set")
        
        print("Dilating...")
        line.dilate(0.0)
        print("Dilation complete")
        
        # Visualization with matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        
        plt.figure(figsize=(10, 10))
        
        # Set the plot limits same as C++ test (-2 to 2)
        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        
        # Plot obstacles as red dots
        obs_x = obs[:, 0]
        obs_y = obs[:, 1]
        plt.scatter(obs_x, obs_y, c='red', s=50, label='Obstacles')
        
        # Plot line segment as red line
        line_pts = line.get_line_segment()
        plt.plot([line_pts[0][0], line_pts[1][0]], 
                [line_pts[0][1], line_pts[1][1]], 
                'r-', linewidth=2, label='Line Segment')
        
        # Plot ellipsoid using sample method
        ellipsoid = line.get_ellipsoid()
        ellipse_pts = np.array(ellipsoid.sample(100))  # Get 100 sample points
        plt.plot(ellipse_pts[:, 0], ellipse_pts[:, 1], 'c-', alpha=0.4, linewidth=2, label='Ellipsoid')
        
        # Plot polyhedron
        poly = line.get_polyhedron()
        vertices = poly.vertices()  # Now calling vertices() as a method
        if len(vertices) > 0:
            # Convert vertices to numpy array for plotting
            vertices_array = np.array(vertices)
            poly_patch = Polygon(vertices_array, alpha=0.2, color='blue', label='Polyhedron')
            plt.gca().add_patch(poly_patch)
        
        plt.grid(True)
        plt.legend()
        plt.title('Line Segment Decomposition')
        plt.axis('equal')
        plt.show()
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()