#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
"""Make a light copy of the robot model, for drawing only.

The RoboParty meshes are CAD quality: 635k triangles in total. Without a GPU the
CPU draws every triangle, so we keep only a part of them for the picture.
Physics still uses the original rpo.xml and meshes, so the robot moves the same.

    make_view_model.py /opt/rpo_description  ->  mjcf/rpo_view.xml + meshes_view/
"""
import os
import struct
import sys

import fast_simplification
import numpy as np

KEEP = 0.08  # keep about 8% of the triangles


def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    count = struct.unpack_from('<I', data, 80)[0]
    if 84 + count * 50 != len(data):
        raise ValueError(f'{path} is not a binary STL file')
    records = np.frombuffer(data, dtype=np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)),
                                                  ('attr', '<u2')]), count=count, offset=84)
    corners = records['v'].reshape(-1, 3)
    points, faces = np.unique(corners, axis=0, return_inverse=True)
    return points, faces.reshape(-1, 3)


def write_stl(path, points, faces):
    tris = points[faces].astype('<f4')
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    records = np.zeros(len(faces), dtype=np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)),
                                                   ('attr', '<u2')]))
    records['normal'] = normals
    records['v'] = tris
    with open(path, 'wb') as f:
        f.write(b'roboparty_sim view mesh'.ljust(80, b' '))
        f.write(struct.pack('<I', len(faces)))
        f.write(records.tobytes())


def main(root):
    src, dst = os.path.join(root, 'meshes'), os.path.join(root, 'meshes_view')
    os.makedirs(dst, exist_ok=True)
    before = after = 0
    for name in sorted(os.listdir(src)):
        if not name.lower().endswith('.stl'):
            continue
        points, faces = read_stl(os.path.join(src, name))
        before += len(faces)
        if len(faces) > 2000:
            points, faces = fast_simplification.simplify(points, faces, target_reduction=1.0 - KEEP)
        write_stl(os.path.join(dst, name), points, faces)
        after += len(faces)
    with open(os.path.join(root, 'mjcf', 'rpo.xml')) as f:
        xml = f.read()
    if 'meshdir="../meshes/"' not in xml:
        raise ValueError('rpo.xml has a different meshdir; update make_view_model.py')
    with open(os.path.join(root, 'mjcf', 'rpo_view.xml'), 'w') as f:
        f.write(xml.replace('meshdir="../meshes/"', 'meshdir="../meshes_view/"'))
    print(f'view model: {before} -> {after} triangles')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/opt/rpo_description')
