#!/usr/bin/env python3
"""Generate convex decomposition for mesh objects using coacd."""

import os
import sys
import trimesh
import coacd
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


# Decomposition parameters
# Adjust these to control the number of convex hulls:
# - threshold: Higher = fewer hulls (0.1-0.5 recommended, default: 0.2)
# - max_convex_hull: Maximum number of hulls per object (default: 5)
DECOMPOSITION_CONFIG = {
    'threshold': 0.3,  # Higher = fewer, more conservative decomposition
    'max_convex_hull': 5,  # Limit maximum hulls per object
}

# Object definitions: (input_file, output_dir, output_prefix)
OBJECTS = [
    (REPO_ROOT / "meshes/onion.stl", REPO_ROOT / "meshes/convex", "onion"),
    (REPO_ROOT / "meshes/tomato.stl", REPO_ROOT / "meshes/convex", "tomato"),
    (REPO_ROOT / "meshes/lettuce.stl", REPO_ROOT / "meshes/convex", "lettuce"),
    (REPO_ROOT / "meshes/knife.stl", REPO_ROOT / "meshes/convex", "knife"),
    (REPO_ROOT / "meshes/plate.stl", REPO_ROOT / "meshes/convex", "plate"),
    (REPO_ROOT / "assets/link_gripper_0.obj", REPO_ROOT / "assets/convex", "gripper"),
]


def decompose_mesh(input_file, output_dir, output_prefix):
    """Decompose a mesh into convex hulls and save them.
    
    Args:
        input_file: Path to input mesh file
        output_dir: Directory to save decomposed meshes
        output_prefix: Prefix for output filenames
    """
    input_path = Path(input_file)
    if not input_path.exists():
        print(f"⚠ Warning: {input_file} not found, skipping...")
        return False
    
    print(f"Processing {input_file}...")
    
    try:
        # Load mesh
        mesh = trimesh.load(str(input_path), force="mesh")
        if not isinstance(mesh, trimesh.Trimesh):
            print(f"  ⚠ Error: Failed to load mesh from {input_file}")
            return False
        
        # Convert to coacd.Mesh
        coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)
        
        # Run convex decomposition with parameters for fewer, larger hulls
        print(f"  Running convex decomposition (threshold={DECOMPOSITION_CONFIG['threshold']}, max_hulls={DECOMPOSITION_CONFIG['max_convex_hull']})...")
        parts = coacd.run_coacd(
            coacd_mesh,
            threshold=DECOMPOSITION_CONFIG['threshold'],
            max_convex_hull=DECOMPOSITION_CONFIG['max_convex_hull'],
            preprocess_mode='auto',
            resolution=2000,
            mcts_nodes=20,
            mcts_iterations=150,
            mcts_max_depth=3,
            merge=True,  # Merge similar hulls
            decimate=False
        )
        print(f"  ✓ Generated {len(parts)} convex hulls")
        
        # Create output directory
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Save each convex hull
        # Each part is a list: [vertices, faces]
        for i, part in enumerate(parts):
            vertices, faces = part[0], part[1]
            part_mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            
            # Save as STL
            output_file = output_path / f"{output_prefix}_convex_{i:03d}.stl"
            part_mesh.export(str(output_file))
            print(f"    Saved: {output_file}")
        
        # Also save a combined mesh for reference
        if len(parts) > 1:
            combined = trimesh.util.concatenate([
                trimesh.Trimesh(vertices=p[0], faces=p[1]) 
                for p in parts
            ])
            combined_file = output_path / f"{output_prefix}_convex_combined.stl"
            combined.export(str(combined_file))
            print(f"    Combined mesh: {combined_file}")
        
        print(f"  ✓ Successfully decomposed {input_file}")
        return True
        
    except Exception as e:
        print(f"  ✗ Error processing {input_file}: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main function to process all objects."""
    print("=" * 60)
    print("Convex Decomposition Script")
    print("=" * 60)
    print()
    
    os.chdir(REPO_ROOT)
    
    success_count = 0
    total_count = len(OBJECTS)
    
    for input_file, output_dir, output_prefix in OBJECTS:
        if decompose_mesh(input_file, output_dir, output_prefix):
            success_count += 1
        print()
    
    print("=" * 60)
    print(f"Summary: {success_count}/{total_count} objects processed successfully")
    print("=" * 60)
    print()
    print("Output locations:")
    print("  - Food items & tools: meshes/convex/")
    print("  - Gripper: assets/convex/")
    print()
    
    if success_count < total_count:
        sys.exit(1)


if __name__ == '__main__':
    main()

