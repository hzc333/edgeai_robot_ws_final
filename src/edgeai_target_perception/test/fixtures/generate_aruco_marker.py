#!/usr/bin/env python3
"""Generate the isolated Gazebo marker model and its image reference."""

from pathlib import Path

import cv2


def main():
    """Generate the ArUco marker model and reference image."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 1, 600, borderBits=1)
    image = cv2.copyMakeBorder(
        marker, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
    model_dir = Path(__file__).resolve().parent / 'aruco_marker_1'
    texture_path = model_dir / 'materials' / 'textures' / 'aruco_id_1.png'
    texture_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(texture_path), image):
        raise RuntimeError(f'failed to write ArUco fixture image: {texture_path}')

    cell_pixels = marker.shape[0] // 6
    cell_size = 0.15 / 6
    black_tiles = []
    for row in range(6):
        for column in range(6):
            sample = marker[
                row * cell_pixels:(row + 1) * cell_pixels,
                column * cell_pixels:(column + 1) * cell_pixels]
            if int(sample.mean()) < 128:
                x = (column - 2.5) * cell_size
                y = (2.5 - row) * cell_size
                black_tiles.append(
                    f"""      <visual name="black_cell_{row}_{column}">
        <pose>{x:.8f} {y:.8f} 0.0005 0 0 0</pose>
        <geometry><box><size>{cell_size:.8f} {cell_size:.8f} 0.001</size></box></geometry>
        <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material>
      </visual>""")

    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="aruco_marker_1">
    <static>true</static>
    <link name="marker">
      <visual name="white_backing">
        <pose>0 0 -0.0005 0 0 0</pose>
        <geometry><box><size>0.15 0.15 0.001</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
{chr(10).join(black_tiles)}
    </link>
  </model>
</sdf>
"""
    model_path = model_dir / 'model.sdf'
    model_path.write_text(model_sdf, encoding='utf-8')
    print(f'Wrote DICT_4X4_50 marker ID 1: {model_path}; reference image: {texture_path}')


if __name__ == '__main__':
    main()
