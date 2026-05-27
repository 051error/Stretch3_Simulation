"""Utility functions for reading anchor positions from XML file."""

import re
import math
from pathlib import Path

import numpy as np

from stretch_sim.paths import get_xml_path


def load_anchors_from_xml(xml_path=None):
    """
    Load anchor positions and directions from MuJoCo XML file.

    Returns:
        dict: {'A': {'pos': [x, y, z], 'direction': angle_rad}, ...}
    """
    if xml_path is None:
        xml_path = get_xml_path()
    xml_path = Path(xml_path)

    if not xml_path.is_file():
        raise FileNotFoundError(f"XML file not found: {xml_path}")
    
    with xml_path.open() as f:
        content = f.read()
    
    anchors = {}
    # Pattern to match both single-letter anchors (A-F) and ORIGIN
    pattern = r'<site\s+name="anchor_([A-Za-z_]+)"[^>]*pos="([^"]+)"'
    
    for match in re.finditer(pattern, content):
        anchor_name = match.group(1).upper()
        pos_str = match.group(2)
        
        # Look for direction in comment
        comment_pattern = rf'<!--\s*anchor_{re.escape(anchor_name)}\s+direction="([^"]+)"\s*-->'
        direction_match = re.search(comment_pattern, content, re.IGNORECASE)
        direction_str = direction_match.group(1) if direction_match else None
        
        try:
            pos_values = [float(x) for x in pos_str.split()]
            if len(pos_values) >= 2:
                pos = pos_values[:3] if len(pos_values) >= 3 else pos_values + [0.0]
                anchor_data = {'pos': pos}
                
                if direction_str:
                    try:
                        anchor_data['direction'] = math.radians(float(direction_str))
                    except ValueError:
                        pass
                
                # Store as "ORIGIN" if it's anchor_ORIGIN, otherwise use single letter
                anchor_key = "ORIGIN" if anchor_name == "ORIGIN" else anchor_name
                anchors[anchor_key] = anchor_data
        except ValueError:
            continue
    
    return anchors


def get_anchor_list(xml_path=None):
    """Get sorted list of available anchor letters."""
    anchors = load_anchors_from_xml(xml_path)
    return sorted(anchors.keys())
