import trimesh
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(script_dir, 'meshes')
os.makedirs(output_dir, exist_ok=True)

# Define your meshes - Update these paths to point to your GLTF files
# Example: Use absolute paths or relative paths from the script directory
meshes = {
    # 'tomato': os.path.join(script_dir, 'path', 'to', 'tomato', 'scene.gltf'),
    # 'onion': os.path.join(script_dir, 'path', 'to', 'onion', 'scene.gltf'),
    # 'cabbage': os.path.join(script_dir, 'path', 'to', 'cabbage', 'scene.gltf'),
    # 'table': os.path.join(script_dir, 'path', 'to', 'dining_table', 'scene.gltf'),
    # 'plate': os.path.join(script_dir, 'path', 'to', 'plate', 'scene.gltf'),
    # 'bell': os.path.join(script_dir, 'path', 'to', 'red_bell_pepper', 'scene.gltf'),
}

for name, gltf_path in meshes.items():
    if os.path.exists(gltf_path):
        print(f"Converting {name}...")
        mesh = trimesh.load(gltf_path)
        
        # Handle scenes with multiple meshes
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(
                [m for m in mesh.geometry.values()]
            )
        
        # Export as OBJ
        output_path = os.path.join(output_dir, f'{name}.obj')
        mesh.export(output_path)
        print(f"  ✓ Saved to {output_path}")
    else:
        print(f"  ✗ {gltf_path} not found")

print("\nDone! Update your XML to use these paths.")

