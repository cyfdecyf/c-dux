#!/usr/bin/env python3
"""Keyboard bottom vs stand: boolean interference + clearance report.

Models the mounted keyboard as solids — PCB (real Edge.Cuts ring), switch
bodies, keycap proxies (box, underside cap-gap above the PCB front face) and
hotswap-socket proxies (box on the PCB back face) — riding the post pattern
(in-plane rotation about the pattern centroid + solver shift/lift, then the
tilt transform). Then:

  1. boolean-intersects it with the full stand / base slab / boot foot
     (all volumes must be 0),
  2. reports parametric clearances (PCB lowest point vs base top, board
     plane vs boot front face, cap underside / socket back vs boot top
     ledge, switch z ranking),
  3. extracts the board corner next to the lowest switch (the global lowest
     PCB material when mounted) with its edge/tip directions,
  4. optionally dumps everything the comparison figure needs (--json).

Proxy sizes are deliberately conservative (oversized); passing here means
the real hardware passes too. Solids are built for the LEFT stand, matching
stand.py's default export.

Usage:
    uv run keyboard_clearance_check.py
    uv run keyboard_clearance_check.py --set-angles -45 --json out/clearance.json
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import re
import sys

from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent))
from build123d import Box, Plane, Polygon, Pos, Rot, extrude

import stand


# ergogen column_net -> key name (see key_layout_check.KEY_NAMES)
KEY_NAMES = {
    'P101': 'pinky_outer', 'P15': 'pinky_bot', 'P18': 'pinky_home',
    'P19': 'pinky_top', 'P20': 'ring_bot', 'P21': 'ring_home',
    'P9': 'ring_TOP', 'P8': 'mid_bot', 'P7': 'mid_home', 'P6': 'mid_top',
    'P4': 'idx_bot', 'P3': 'idx_home', 'P0': 'idx_top', 'P5': 'inner_bot',
    'P2': 'inner_home', 'P1': 'inner_top', 'P14': 'thumb_home',
    'P16': 'thumb_near', 'P10': 'thumb_far',
}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--tilt', type=float, default=stand.TILT)
    ap.add_argument(
        '--set-angles',
        default=','.join(str(a) for a in stand.SET_ANGLES),
        help='comma list (first angle is used for the single-set checks);'
             ' leading minus needs the = form: --set-angles=-45',
    )
    ap.add_argument('--post-h', type=float, default=3.0)
    ap.add_argument('--switch-size', type=float, default=14.5)
    ap.add_argument('--switch-h', type=float, default=7.5)
    ap.add_argument('--cap-size', type=float, default=18.0)
    ap.add_argument('--cap-gap', type=float, default=3.5,
                    help='cap underside above the PCB front face')
    ap.add_argument('--cap-h', type=float, default=5.5)
    ap.add_argument('--socket-size', type=float, default=14.0)
    ap.add_argument('--socket-depth', type=float, default=12.0)
    ap.add_argument('--socket-t', type=float, default=2.2,
                    help='socket body proud of the PCB back face')
    ap.add_argument('--pcb-t', type=float, default=1.6)
    ap.add_argument('--json', metavar='PATH',
                    help='dump figure-ready geometry data to PATH')
    return ap.parse_args()


def edge_ring(text: str) -> list[tuple[float, float]]:
    """Edge.Cuts segments chained into closed loops; returns the outer ring
    in the stand frame (u = Y_max - y, v = x - X_min). Arcs are already
    tessellated into short lines by ergogen, so endpoints suffice."""
    segs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for line in text.splitlines():
        if 'Edge.Cuts' not in line or not line.strip().startswith('(gr_'):
            continue
        xy = [(float(a), float(b)) for a, b in re.findall(
            r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line)]
        if len(xy) >= 2:
            segs.append((xy[0], xy[1]))
    x_min = min(min(p[0] for p in s) for s in segs)
    y_max = max(max(p[1] for p in s) for s in segs)
    segs = [((y_max - y1, x1 - x_min), (y_max - y2, x2 - x_min))
            for (x1, y1), (x2, y2) in segs]

    rings: list[list[tuple[float, float]]] = []
    pool = list(segs)
    while pool:
        ring = [pool[0][0]]
        cur = pool[0][1]
        pool.pop(0)
        while True:
            hit = next((i for i, (p, q) in enumerate(pool)
                        if math.dist(cur, p) < 0.01
                        or math.dist(cur, q) < 0.01), None)
            if hit is None:
                break
            p, q = pool.pop(hit)
            cur = q if math.dist(cur, p) < math.dist(cur, q) else p
            if math.dist(cur, ring[0]) < 0.01:
                break
            ring.append(cur)
        rings.append(ring)
    rings.sort(key=len, reverse=True)
    return rings[0]


def switch_positions(
    text: str,
) -> dict[str, tuple[float, float, str]]:
    """Switch centres in the stand frame (u, v) + footprint ref (same
    framing as parse_pcb / key_layout_check)."""
    x_min = min(
        float(a)
        for line in text.splitlines()
        if 'Edge.Cuts' in line and line.strip().startswith('(gr_')
        for a, _ in re.findall(
            r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line)
    )
    y_max = max(
        float(b)
        for line in text.splitlines()
        if 'Edge.Cuts' in line and line.strip().startswith('(gr_')
        for _, b in re.findall(
            r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line)
    )
    nets = dict(re.findall(r'\(net (\d+) "([^"]*)"\)', text))
    keys: dict[str, tuple[float, float, str]] = {}
    for chunk in text.split('(footprint ')[1:]:
        if not re.match(r'"[^"]*:switch_choc_v1_v2"', chunk):
            continue
        ref = re.search(r'\(property "Reference" "([^"]+)"', chunk).group(1)
        at = re.search(r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)', chunk)
        pnets = [nets.get(n) for n in re.findall(r'\(net (\d+) "', chunk)]
        col = sorted(
            (p for p in pnets if p and p.startswith('P')), key=lambda s: int(s[1:])
        )[0]
        x, y = float(at.group(1)), float(at.group(2))
        keys[KEY_NAMES[col]] = (y_max - y, x - x_min, ref)
    return keys


def main() -> None:
    args = parse_args()
    angles = [float(a) for a in args.set_angles.split(',')]
    with contextlib.redirect_stdout(io.StringIO()):
        pcb_w, pcb_h, holes, outline = stand.parse_pcb(stand.DEFAULT_PCB)
        s = stand.Stand(pcb_w, pcb_h, holes, outline, args.post_h, 'nut',
                        angles, tilt=args.tilt)
    ang_, s_shift, lift, corner_z = s.set_meta[0]
    ct, st, face_z = s.ct, s.st, s.face_z
    xform = Pos(0, 0, s.origin_z) * Rot(X=90 - s.tilt)
    print(f'tilt {s.tilt:g} / set-angles {ang_:+g}: shift {s_shift:g}, '
          f'lift {lift:g}, solver lowest outline z {corner_z}')

    text = stand.DEFAULT_PCB.read_text()
    ring = edge_ring(text)
    keys = switch_positions(text)
    holes_uv = dict(zip(sorted(stand.HOLES_EXPECTED), holes, strict=True))
    cu = sum(u for u, _ in holes) / len(holes)
    cv = sum(v for _, v in holes) / len(holes)
    r = math.radians(ang_)

    def to_plate(u: float, v: float) -> tuple[float, float]:
        """stand frame -> plate frame (board rides the rotated pattern)."""
        ru = cu + (u - cu) * math.cos(r) - (v - cv) * math.sin(r)
        rv = cv + (u - cu) * math.sin(r) + (v - cv) * math.cos(r)
        return (ru - pcb_w / 2 + s_shift,
                rv + stand.bottom_ext - stand.keyboard_drop + lift)

    def to_world(
        x_l: float, y_l: float, z_l: float,
    ) -> tuple[float, float, float]:
        return (x_l, st * y_l - ct * z_l,
                s.origin_z + ct * y_l + st * z_l)

    def pdist(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # ---- solids ------------------------------------------------------------
    board_pts = [to_plate(u, v) for u, v in ring]
    keyboard = xform * extrude(
        Plane.XY.offset(face_z) * Polygon(*board_pts), args.pcb_t)
    hs, cd, _sw = args.cap_size / 2, args.cap_gap, args.switch_size / 2
    for _name, (u, v, _ref) in keys.items():
        x, y = to_plate(u, v)
        keyboard += xform * (Pos(x, y, face_z + args.pcb_t + args.switch_h / 2)
                             * Box(args.switch_size, args.switch_size,
                                   args.switch_h))
        keyboard += xform * (Pos(x, y,
                                 face_z + args.pcb_t + cd + args.cap_h / 2)
                             * Box(args.cap_size, args.cap_size, args.cap_h))
        keyboard += xform * (Pos(x, y, face_z - args.socket_t / 2)
                             * Box(args.socket_size, args.socket_depth,
                                   args.socket_t))

    stand_full = s.base_solid() + s.upright_solid()
    boot = Pos(s.boot_lo, 0, 0) * extrude(
        Plane.YZ * Polygon((s.boot_front, stand.base_t),
                           (s.boot_rear, stand.base_t),
                           (s.boot_rear, stand.base_t + stand.boot_h),
                           (s.boot_front + stand.boot_h * st / ct,
                            stand.base_t + stand.boot_h)),
        s.boot_hi - s.boot_lo)

    print('\n== 1. boolean interference (volume, expect 0)')
    for label, other in (('full stand', stand_full),
                         ('base slab', s.base_solid()), ('boot foot', boot)):
        vol = (keyboard & other).volume / 1000
        print(f'   keyboard ∩ {label:<11s}: {vol:.4f} cm³')

    # ---- clearances --------------------------------------------------------
    print('\n== 2. clearances')
    zs = [to_world(*to_plate(u, v), face_z)[2] for u, v in ring]
    i_min = min(range(len(ring)), key=lambda i: zs[i])
    low_uv = ring[i_min]
    low_w = to_world(*to_plate(*low_uv), face_z)
    near = min(keys.items(),
               key=lambda kv: pdist(to_plate(kv[1][0], kv[1][1]),
                                    to_plate(*low_uv)))
    near_d = pdist(to_plate(near[1][0], near[1][1]), to_plate(*low_uv))
    print(f'   PCB lowest ring point (u,v)=({low_uv[0]:.1f},{low_uv[1]:.1f}) '
          f'-> world ({low_w[0]:.1f},{low_w[1]:.1f}) z={low_w[2]:.2f}; '
          f'gap to base top {low_w[2] - stand.base_t:.2f} mm')
    print(f'   nearest switch: {near[0]} ({near[1][2]}), board-plane dist '
          f'{near_d:.1f} mm')
    print(f'   board back face vs boot front face: {s.post_h:g} mm '
          '(post_h; boot front = plate front plane)')
    zsw = sorted((to_world(*to_plate(u, v), face_z)[2], name, ref)
                 for name, (u, v, ref) in keys.items())
    print('   lowest switches (world z at switch centre):')
    for z, name, ref in zsw[:4]:
        print(f'     {name:<12s} ({ref})  z = {z:6.2f}')
    ledge = stand.base_t + stand.boot_h
    worst_cap = worst_sock = None
    for _name, (u, v, _ref) in keys.items():
        x, y = to_plate(u, v)
        if min(x + hs, s.boot_hi) - max(x - hs, s.boot_lo) <= 0:
            continue
        z_cap = s.origin_z + ct * (y - hs) + st * (face_z + args.pcb_t + cd)
        z_sock = (s.origin_z + ct * (y - args.socket_depth / 2)
                  + st * (face_z - args.socket_t))
        if worst_cap is None or z_cap < worst_cap[0]:
            worst_cap = (z_cap, name)
        if worst_sock is None or z_sock < worst_sock[0]:
            worst_sock = (z_sock, name)
    print(f'   keycap underside, lowest corner above base top: '
          f'{worst_cap[0] - stand.base_t:.2f} mm  ({worst_cap[1]})')
    print(f'   socket back plane, lowest corner vs boot top ledge z={ledge}: '
          f'{worst_sock[0] - ledge:+.2f} mm  ({worst_sock[1]}; socket body '
          f'{args.socket_t:g} mm proud of PCB back, still '
          f'{s.post_h - args.socket_t:.1f} mm clear of boot face)')

    # ---- lowest board corner (the global lowest PCB material point) --------
    print('\n== 3. lowest board corner (lowest PCB material when mounted)')
    s_lo = keys[near[0]]
    s_lo_pl = to_plate(s_lo[0], s_lo[1])
    vi = i_min
    vert = ring[vi]
    corn_w = to_world(*to_plate(*vert), face_z)
    n = len(ring)
    prev_v, next_v = ring[(vi - 1) % n], ring[(vi + 1) % n]
    print(f'   switch {near[0]} plate ({s_lo_pl[0]:.1f},{s_lo_pl[1]:.1f}), '
          f'nearest corner vertex (u,v)=({vert[0]:.2f},{vert[1]:.2f}) -> '
          f'world ({corn_w[0]:.1f},{corn_w[1]:.1f},{corn_w[2]:.2f}), '
          f'board-plane dist {pdist(to_plate(*vert), s_lo_pl):.1f} mm')
    print(f'   gap to base top {corn_w[2] - stand.base_t:.2f} mm; '
          f'x within boot span: {s.boot_lo <= corn_w[0] <= s.boot_hi}')

    # ---- H3 self-tap screw capacity (informational only) --------------------
    # The 3 nut-clamped screws hold the board in practice; this hole is
    # rarely used, so the report just states what length fits — no gating.
    print('\n== 4. H3 self-tap pilot: usable screw length (informational)')
    hx, hy = s.lowest_post
    free = face_z + args.pcb_t            # bore through PCB+post+plate
    x0, x1 = (s.band_mid - stand.key_x - stand.key_l / 2 - stand.joint_clear,
              s.band_mid + stand.key_x + stand.key_l / 2 + stand.joint_clear)
    in_groove_x = x0 <= hx <= x1
    # the boot/haunch plastic behind the back face ends at the key-groove
    # front wall (world y = key_y0 - joint_clear) when the post sits over
    # the groove x-span; otherwise the boot prism bounds it instead
    y_wall = (stand.key_y0 - stand.joint_clear if in_groove_x
              else s.boot_rear)
    toe = (y_wall - st * hy) / ct
    seat = (ct * hy) / st                # axis crosses the seat plane z=6
    toe = min(toe, seat)
    print(f'   free bore Ø{stand.tap_d:g} (PCB {args.pcb_t:g} + post '
          f'{s.post_h:g} + plate {stand.plate_t:g}) = {free:.1f} mm')
    print(f'   toe plastic behind back face (ends at '
          f"{'groove front wall' if in_groove_x else 'boot rear'} y={y_wall:g})"
          f' = {toe:.1f} mm')
    print(f'   max M4 length under head  ~ {free + toe:.1f} mm  '
          '(tip must stay in plastic; beyond it enters the key groove and '
          'hits the base key)')
    for screw_len in (12, 14, 16):
        bite = screw_len - free   # tip depth into the toe plastic
        print(f'   M4x{screw_len}: tip bites {bite:.1f} mm into the toe '
              'plastic')

    if args.json:
        fig = {
            'tilt': s.tilt, 'set_angle': ang_, 'shift': s_shift, 'lift': lift,
            'pcb_w': pcb_w, 'pcb_h': pcb_h, 'face_z': face_z,
            'origin_z': s.origin_z, 'ct': ct, 'st': st,
            'ring': ring, 'outline_samples': outline,
            'keys': {k: {'uv': [u, v], 'ref': ref}
                     for k, (u, v, ref) in keys.items()},
            'holes': {ref: list(h) for ref, h in holes_uv.items()},
            'posts': s.all_posts,
            'base': {'t': stand.base_t, 'lo': s.slab_lo, 'hi': s.slab_hi,
                     'front': -stand.base_front, 'rear': stand.base_rear},
            'boot': {'lo': s.boot_lo, 'hi': s.boot_hi,
                     'front': s.boot_front, 'rear': s.boot_rear,
                     'top': ledge},
            'lowest_ring_pt': {'uv': list(low_uv), 'world': list(low_w),
                               'z': round(low_w[2], 2),
                               'nearest_key': near[0]},
            'ring_top_corner': {'idx': vi, 'uv': list(vert),
                                'world': list(corn_w),
                                'prev_uv': list(prev_v),
                                'next_uv': list(next_v)},
            'switch_z': sorted([[round(z, 2), name, ref]
                                for z, name, ref in zsw]),
        }
        Path(args.json).write_text(json.dumps(fig))
        print(f'\nfigure data -> {args.json}'
              ' (render with pcb_rotation_figure.py)')


if __name__ == '__main__':
    main()
