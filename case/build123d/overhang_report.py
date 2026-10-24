"""One-shot overhang audit for an exported STL (default: right upright t30).

Loads the binary STL, computes every facet's normal, and reports all
downward-facing area grouped by slope-from-horizontal (0 deg = flat
ceiling, worst; 90 deg = vertical wall, free). Facets below the 45 deg
support threshold are clustered via shared edges so each problem spot
can be located and sized in mm^2.

Usage: uv run --with numpy python overhang_report.py [file.stl]
"""

import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

BUCKETS = [(0, 10), (10, 30), (30, 45), (45, 60), (60, 90)]
THRESHOLD = 45.0  # slope below this gets support in most slicers


def load_stl(path: Path) -> np.ndarray:
    with open(path, 'rb') as f:
        header = f.read(80)
        assert not header.lstrip().startswith(b'solid'), 'ASCII STL not supported'
        (n,) = struct.unpack('<I', f.read(4))
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        return raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3).astype(np.float64)


def main() -> None:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else 'out/stand_right_upright_t30.stl')
    tris = load_stl(path)
    n = len(tris)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    areas = np.linalg.norm(normals, axis=1) / 2
    nz = normals[:, 2] / (2 * areas)  # unit normal z
    slope = np.degrees(np.arccos(np.clip(np.abs(nz), -1, 1)))  # 0=ceiling, 90=wall

    total = areas.sum()
    print(f'{path.name}: {n} facets, total area {total / 100:.1f} cm^2')
    down = nz < 0
    print(f'downfacing area: {areas[down].sum() / 100:.1f} cm^2 '
          f'({100 * areas[down].sum() / total:.0f}% of total)')
    for lo, hi in BUCKETS:
        m = down & (slope >= lo) & (slope < hi)
        print(f'  slope {lo:2d}-{hi:2d} deg: {areas[m].sum() / 100:6.2f} cm^2')

    # cluster the risky (sub-threshold, downfacing) facets via shared edges
    risky = np.where(down & (slope < THRESHOLD))[0]
    key = np.round(tris.reshape(-1, 3), 3)
    vid = {}
    ids = np.empty(key.shape[0], dtype=np.int64)
    for i, v in enumerate(map(tuple, key)):
        ids[i] = vid.setdefault(v, len(vid))
    ids = ids.reshape(n, 3)

    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    edge_owner = defaultdict(list)
    for f in risky:
        a, b, c = ids[f]
        for e in ((a, b), (b, c), (c, a)):
            edge_owner[tuple(sorted(e))].append(f)
    for owners in edge_owner.values():
        if len(owners) > 1:
            r = find(owners[0])
            for f in owners[1:]:
                parent[find(f)] = r

    clusters = defaultdict(list)
    for f in risky:
        clusters[find(f)].append(f)
    print(f'\nsub-45deg downfacing area: {areas[risky].sum() / 100:.2f} cm^2 '
          f'in {len(clusters)} clusters; biggest (mm^2):')
    ranked = sorted(clusters.values(), key=lambda fs: -areas[fs].sum())
    for fs in ranked[:12]:
        pts = tris[fs].reshape(-1, 3)
        mn, mx = pts.min(0), pts.max(0)
        w = areas[fs].sum()
        dom = slope[fs].mean()
        print(f'  {w:7.1f} mm^2  mean slope {dom:4.0f} deg  '
          f'x[{mn[0]:7.1f},{mx[0]:7.1f}] y[{mn[1]:6.1f},{mx[1]:6.1f}] '
          f'z[{mn[2]:6.1f},{mx[2]:6.1f}]')

    # sanity: largest flat ceiling = bed-contact face should NOT be here
    flat = np.where(down & (slope < 1))[0]
    if len(flat):
        print('\nhorizontal ceilings (true bridges):')
        for f in flat[:1]:
            pass
        m = down & (slope < 1)
        print(f'  total {areas[m].sum() / 100:.2f} cm^2 across {len(flat)} facets')


if __name__ == '__main__':
    main()
