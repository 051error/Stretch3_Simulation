import trimesh
import numpy as np
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
mesh_path = os.path.join(script_dir, 'meshes', 'table_fixed.obj')

# Load the table mesh
mesh = trimesh.load(mesh_path)
volume = mesh.volume
print(f"Mesh volume: {volume} m³")
mass = 0.15  # 150 grams

# Calculate density
density = mass / volume
print(f"Density: {density} kg/m³")
print("Table Mesh Analysis:")
print("=" * 50)
print(f"Bounding box min: {mesh.bounds[0]}")
print(f"Bounding box max: {mesh.bounds[1]}")
print(f"Dimensions (X, Y, Z): {mesh.extents}")
print(f"Center/Centroid: {mesh.centroid}")
print(f"Number of vertices: {len(mesh.vertices)}")
print(f"Number of faces: {len(mesh.faces)}")

# Check if mesh needs to be reoriented
print(f"\nMesh likely needs adjustment if:")
print(f"  - Z-center is far from 0: {abs(mesh.centroid[2]) > 1}")
print(f"  - Any dimension > 10m: {any(mesh.extents > 10)}")
print(f"  - Min Z is not near 0: {abs(mesh.bounds[0][2]) > 0.1}")