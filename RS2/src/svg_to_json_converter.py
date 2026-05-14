#!/usr/bin/env python3
"""
SVG to JSON stroke converter with round-trip verification.

Converts SVG path data to stroke coordinates (JSON format),
then converts back to SVG for visual verification.
"""

import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple

Stroke = List[Tuple[float, float]]
Strokes = List[Stroke]


def parse_svg_path_data(d_string: str) -> Stroke:
    """
    Parse SVG path 'd' attribute into a list of (x, y) coordinates.
    Handles only M (moveto) and C (cubic bezier), L (lineto) commands.
    Approximates bezier curves with line segments.
    """
    points = []
    
    # Tokenize the path data
    # Split by spaces and commas, but keep command letters
    tokens = re.findall(r'[MmLlCcSsQqTtAaZz]|[-\d.]+', d_string)
    
    current_x, current_y = 0, 0
    start_x, start_y = 0, 0
    i = 0
    
    while i < len(tokens):
        token = tokens[i]
        
        if token in 'Mm':  # Move commands
            is_relative = token == 'm'
            x = float(tokens[i + 1])
            y = float(tokens[i + 2])
            
            if is_relative:
                current_x += x
                current_y += y
            else:
                current_x = x
                current_y = y
            
            points.append((current_x, current_y))
            start_x, start_y = current_x, current_y
            i += 3
            
        elif token in 'Ll':  # Line commands
            is_relative = token == 'l'
            x = float(tokens[i + 1])
            y = float(tokens[i + 2])
            
            if is_relative:
                current_x += x
                current_y += y
            else:
                current_x = x
                current_y = y
            
            points.append((current_x, current_y))
            i += 3
            
        elif token in 'Cc':  # Cubic bezier
            is_relative = token == 'c'
            # Bezier: control1_x, control1_y, control2_x, control2_y, end_x, end_y
            x1 = float(tokens[i + 1])
            y1 = float(tokens[i + 2])
            x2 = float(tokens[i + 3])
            y2 = float(tokens[i + 4])
            x = float(tokens[i + 5])
            y = float(tokens[i + 6])
            
            if is_relative:
                x1 += current_x
                y1 += current_y
                x2 += current_x
                y2 += current_y
                x += current_x
                y += current_y
            
            # Approximate bezier with 10 line segments
            for t in [j / 10.0 for j in range(1, 11)]:
                # Cubic bezier formula
                mt = 1 - t
                bx = (mt**3 * current_x + 
                      3 * mt**2 * t * x1 + 
                      3 * mt * t**2 * x2 + 
                      t**3 * x)
                by = (mt**3 * current_y + 
                      3 * mt**2 * t * y1 + 
                      3 * mt * t**2 * y2 + 
                      t**3 * y)
                points.append((bx, by))
            
            current_x = x
            current_y = y
            i += 7
            
        elif token in 'Zz':  # Close path
            if points and (points[-1][0] != start_x or points[-1][1] != start_y):
                points.append((start_x, start_y))
            i += 1
            
        else:
            i += 1
    
    return points


def svg_to_json(svg_path: str, output_json_path: str = None) -> Tuple[Strokes, str]:
    """
    Convert SVG file to JSON stroke format.
    
    Args:
        svg_path: Path to SVG file
        output_json_path: Optional output path (default: same name with .json)
    
    Returns:
        Tuple of (strokes list, output file path)
    """
    if output_json_path is None:
        output_json_path = str(Path(svg_path).with_suffix('.json'))
    
    # Parse SVG
    tree = ET.parse(svg_path)
    root = tree.getroot()
    
    # Handle SVG namespace
    namespaces = {'svg': 'http://www.w3.org/2000/svg'}
    
    # Find all path elements
    paths = root.findall('.//svg:path', namespaces)
    if not paths:
        paths = root.findall('.//path')  # Fallback if no namespace
    
    strokes = []
    for path_elem in paths:
        d = path_elem.get('d', '')
        if d:
            stroke = parse_svg_path_data(d)
            if stroke:  # Only add non-empty strokes
                strokes.append(stroke)
    
    # Write JSON
    with open(output_json_path, 'w') as f:
        json.dump(strokes, f, indent=2)
    
    print(f"✓ Converted SVG → JSON")
    print(f"  Strokes: {len(strokes)}")
    print(f"  Total points: {sum(len(s) for s in strokes)}")
    print(f"  Output: {output_json_path}")
    
    return strokes, output_json_path


def json_to_svg(strokes: Strokes, output_svg_path: str, width: int = 800, height: int = 600) -> str:
    """
    Convert JSON stroke format back to SVG for verification.
    
    Args:
        strokes: List of strokes (each stroke is list of (x,y) tuples)
        output_svg_path: Output SVG file path
        width: SVG canvas width
        height: SVG canvas height
    
    Returns:
        Path to output SVG
    """
    # Create SVG root
    svg_root = ET.Element('svg', {
        'width': str(width),
        'height': str(height),
        'xmlns': 'http://www.w3.org/2000/svg'
    })
    
    # Create group for paths
    g = ET.SubElement(svg_root, 'g')
    
    # Convert each stroke to SVG path
    for stroke_idx, stroke in enumerate(strokes):
        if not stroke:
            continue
        
        # Build path data string
        path_data = f"M {stroke[0][0]},{stroke[0][1]}"
        for x, y in stroke[1:]:
            path_data += f" L {x},{y}"
        
        # Create path element
        ET.SubElement(g, 'path', {
            'd': path_data,
            'stroke': '#000000',
            'stroke-width': '1',
            'fill': 'none',
            'id': f'stroke_{stroke_idx}'
        })
    
    # Write SVG
    tree = ET.ElementTree(svg_root)
    tree.write(output_svg_path)
    
    print(f"✓ Converted JSON → SVG")
    print(f"  Strokes: {len(strokes)}")
    print(f"  Canvas: {width}×{height}")
    print(f"  Output: {output_svg_path}")
    
    return output_svg_path


def main():
    """
    Main conversion pipeline with verification.
    
    Expected directory structure:
    /home/domenic/RS2/
    ├── inputs/
    │   └── (place SVG files here)
    ├── outputs/
    │   ├── strokes/
    │   └── verified/
    └── src/
        └── svg_to_json_converter.py (this file)
    
    Run from parent directory: cd .. && python3 src/svg_to_json_converter.py
    """
    
    import os
    
    # Use relative paths from parent directory
    svg_name = 'face3'  # Will look for face1.svg in inputs/
    svg_input = f'inputs/{svg_name}.svg'
    json_output = f'outputs/strokes/{svg_name}_strokes.json'
    svg_verify = f'outputs/verified/{svg_name}_verified.svg'
    
    print("\n" + "="*60)
    print("SVG → JSON → SVG Converter")
    print("="*60 + "\n")
    
    # Step 1: SVG to JSON
    print("[Step 1] Converting SVG to JSON strokes...")
    strokes, json_path = svg_to_json(svg_input, json_output)
    
    print("\n[Step 2] Converting JSON back to SVG for verification...")
    # Step 2: JSON back to SVG
    svg_verify_path = json_to_svg(strokes, svg_verify)
    
    print("\n" + "="*60)
    print("Verification Status")
    print("="*60)
    print(f"✓ Original SVG:  {svg_input}")
    print(f"✓ JSON output:   {json_path}")
    print(f"✓ Verify SVG:    {svg_verify_path}")
    print(f"\nStrokes extracted: {len(strokes)}")
    for i, stroke in enumerate(strokes):
        print(f"  Stroke {i+1}: {len(stroke)} points")
    
    # Print JSON sample
    print(f"\nJSON Format Sample (first 3 points of stroke 1):")
    if strokes and len(strokes[0]) > 0:
        print(f"  {strokes[0][:3]}")
    
    print("\n" + "="*60)
    print("Done! Open the SVG files to visually compare:")
    print(f"  Original: {svg_input}")
    print(f"  Verified: {svg_verify_path}")
    print(f"\nDirectory structure:")
    print(f"  inputs/        ← Put your SVG files here")
    print(f"  outputs/strokes/       ← JSON strokes output")
    print(f"  outputs/verified/      ← Verification SVGs")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()
