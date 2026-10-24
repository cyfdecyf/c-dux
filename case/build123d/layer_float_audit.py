"""Slicer-independent float audit: for each 0.2 mm layer, compute the layer
cross-section area that extends beyond the previous layer dilated by half a
line width. Reports per-layer float stats, the worst layers (with plan-view
location), and any layer containing a component that starts entirely in air
(true failure risk). Section loops are projected to XY directly and combined
with the even-odd rule so the frame is consistent across layers and holes
stay holes.

Usage: uv run --with trimesh --with shapely --with numpy python layer_float_audit.py file.stl
"""

import sys

import numpy as np
import trimesh

from shapely.geometry import Polygon


LAYER = 0.2
HALF_LINE = 0.25  # supported zone = prev layer + ~half extrusion width


def layer_polygon(mesh, z):
    sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if sec is None:
        return None
    loops = sec.discrete
    if len(loops) == 0:
        return None
    acc = None
    for lp in loops:
        p = Polygon(lp[:, :2])
        if not p.is_valid:
            p = p.buffer(0)
        acc = p if acc is None else acc.symmetric_difference(p)
    return acc if not acc.is_empty else None


def main():
    mesh = trimesh.load(sys.argv[1])
    z0, z1 = mesh.bounds[0][2], mesh.bounds[1][2]
    zs = np.arange(z0 + LAYER / 2, z1, LAYER)
    print(f'{sys.argv[1]}: z {z0:.1f}..{z1:.1f}, {len(zs)} layers @ {LAYER} mm')
    com = mesh.center_mass
    print(f'center of mass: x={com[0]:.1f} y={com[1]:.1f} z={com[2]:.1f} '
          f'(mass {mesh.volume * 1.27 / 1000:.0f} g PETG solid)')

    prev = None
    floats = []
    islands = []
    for z in zs:
        cur = layer_polygon(mesh, z)
        if cur is None or cur.is_empty:
            prev = None
            continue
        if prev is None or prev.is_empty:
            prev = cur
            continue
        supported = prev.buffer(HALF_LINE, join_style=2)
        fl = cur.difference(supported)
        a = fl.area if not fl.is_empty else 0.0
        if a > 0:
            c = fl.centroid if fl.geom_type == 'Polygon' else fl.representative_point()
            floats.append((z, a, c.x, c.y))
        for comp in cur.geoms if cur.geom_type == 'MultiPolygon' else [cur]:
            if comp.intersection(prev).is_empty and comp.distance(prev) > 0.6:
                islands.append((z, comp.area))
        prev = cur

    arr = np.array([(z, a) for z, a, _, _ in floats])
    total = arr[:, 1].sum()
    print(f'layers with float: {(arr[:, 1] > 0.01).sum()}/{len(arr)}')
    print(f'total float area: {total / 100:.2f} cm^2 '
          f'(= {total * LAYER / 1000:.2f} cm^3 of droop-prone solid)')
    print('worst layers (z, area, plan x/y of float):')
    for z, a, cx, cy in sorted(floats, key=lambda t: -t[1])[:10]:
        print(f'  z={z:7.2f}  {a:7.1f} mm^2   at x={cx:7.1f} y={cy:7.1f}')
    big = arr[arr[:, 1] > 5]
    if len(big):
        print(f'layers >5 mm^2 float: {len(big)}, z range '
              f'{big[:, 0].min():.1f}..{big[:, 0].max():.1f} mm')
    if islands:
        print('AIR-START ISLANDS (true failure risk):')
        for z, a in islands:
            print(f'  z={z:7.2f}  area {a:7.1f} mm^2')
    else:
        print('no air-start islands: every layer is anchored to the one below')


if __name__ == '__main__':
    main()
