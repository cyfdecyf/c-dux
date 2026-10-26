#!/usr/bin/env python3
"""Which switch ends up lowest once the board is mounted?

Solid-free quick query: for every --set-angles value the post solver is run,
the whole board (switch centres included) rides the in-plane rotation about
the pattern centroid, and the world height of every switch is listed. Use it
to pick the rotation direction or sanity-check the key layout before any CAD
gets built. Numbers only — for solid interference / clearances run
keyboard_clearance_check.py.

Usage:
    uv run key_layout_check.py                     # default stand angles
    uv run key_layout_check.py --set-angles -40,-45,45
"""

from __future__ import annotations

import argparse
import contextlib
import io
import math
import re
import sys

from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent))
import stand


# ergogen column_net -> key name (columns/rows per ergogen/config.yaml)
KEY_NAMES = {
    'P101': 'pinky_outer/home',
    'P15': 'pinky/bottom',
    'P18': 'pinky/home',
    'P19': 'pinky/top',
    'P20': 'ring/bottom',
    'P21': 'ring/home',
    'P9': 'ring/TOP',
    'P8': 'middle/bottom',
    'P7': 'middle/home',
    'P6': 'middle/top',
    'P4': 'index/bottom',
    'P3': 'index/home',
    'P0': 'index/top',
    'P5': 'inner/bottom',
    'P2': 'inner/home',
    'P1': 'inner/top',
    'P14': 'thumb/home',
    'P16': 'thumb/near',
    'P10': 'thumb/far',
}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        '--tilt', type=float, default=stand.TILT, help=f'={stand.TILT:g}'
    )
    ap.add_argument(
        '--set-angles',
        default=','.join(str(a) for a in stand.SET_ANGLES),
        help='comma list of in-plane angles, deg; leading minus needs'
             ' the = form: --set-angles=-40,-45',
    )
    ap.add_argument(
        '--all', action='store_true', help='list every switch, not just 3'
    )
    return ap.parse_args()


def switch_positions(text: str) -> dict[str, tuple[float, float, str]]:
    """Switch centres in the stand frame (u, v) + footprint ref, from the
    KiCad file the same way parse_pcb frames the mounting holes."""
    xs, ys = [], []
    for line in text.splitlines():
        if 'Edge.Cuts' in line and line.strip().startswith('(gr_'):
            for a, b in re.findall(
                r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line
            ):
                xs.append(float(a))
                ys.append(float(b))
    x_min, y_max = min(xs), max(ys)

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
    pcb_w, pcb_h, holes, outline = stand.parse_pcb(stand.DEFAULT_PCB)
    keys = switch_positions(stand.DEFAULT_PCB.read_text())
    cu = sum(u for u, _ in holes) / len(holes)
    cv = sum(v for _, v in holes) / len(holes)
    face_z = stand.plate_t + 3  # default post_h

    for ang in [float(a) for a in args.set_angles.split(',')]:
        with contextlib.redirect_stdout(io.StringIO()):
            s = stand.Stand(
                pcb_w, pcb_h, holes, outline, 3, 'nut', [ang], tilt=args.tilt
            )
        ang_, shift, lift, corner_z = s.set_meta[0]
        r = math.radians(ang_)
        rows = []
        for name, (u, v, ref) in keys.items():
            ru = cu + (u - cu) * math.cos(r) - (v - cv) * math.sin(r)
            rv = cv + (u - cu) * math.sin(r) + (v - cv) * math.cos(r)
            y_l = rv + stand.bottom_ext - stand.keyboard_drop + lift
            z_w = stand.base_t + y_l * s.ct + face_z * s.st
            rows.append((z_w, ru - pcb_w / 2 + shift, y_l, name, ref))
        rows.sort()
        shown = rows if args.all else rows[:3]
        print(f'\nset-angles {ang_:+g} (tilt {args.tilt:g}): '
              f'lowest PCB outline point z = {corner_z}')
        print(f'  lowest switch = {rows[0][3]} ({rows[0][4]}), '
              f'z = {rows[0][0]:.2f}')
        print('  switch heights, low -> high:'
              if args.all
              else '  3 lowest switches:')
        for z_w, x_l, y_l, name, ref in shown:
            print(f'    {name:<18s} ({ref})  z = {z_w:6.2f}  '
                  f'plate x = {x_l:7.2f}  y = {y_l:7.2f}')


if __name__ == '__main__':
    main()
