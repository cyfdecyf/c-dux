#!/usr/bin/env python3
"""Mounting-orientation diagram for the README (mounting.png).

Two panels, both true to the current single-post-group design:
  left — the PCB flat in the stand frame (no in-plane rotation): real
         Edge.Cuts outline, switches, MCU/JST/encoder, the 4 mounting holes,
         pinky-stalk-down / thumb-up orientation and the u/v mapping;
  right — the same board riding the post pattern (rotated about the pattern
          centroid by --set-angles, solver shift/lift applied) on the actual
          backplate outline, extracted as a mid-thickness section of
          plate_and_posts(); H3 annotated as the self-tapping pilot.

Usage:
    uv run mounting_diagram.py                     # defaults: t30 / -45
    uv run mounting_diagram.py --set-angles=-45 --out images/mounting.png
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
import matplotlib

from build123d import Box, Pos

import stand

from keyboard_clearance_check import edge_ring, switch_positions


matplotlib.use('Agg')
import matplotlib.pyplot as plt

from matplotlib.patches import Polygon as MplPolygon


plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK HK',
                                   'WenQuanYi Zen Hei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

C_BOARD, C_EDGE, C_KEY = '#cfe3f7', '#2b6cb0', '0.45'
C_POST, C_TAP, C_MCU = '#e8590c', '#d6336c', '0.15'


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--tilt', type=float, default=stand.TILT)
    ap.add_argument(
        '--set-angles',
        default=','.join(str(a) for a in stand.SET_ANGLES),
        help='leading minus needs the = form: --set-angles=-45',
    )
    ap.add_argument('--out', default='mounting.png')
    return ap.parse_args()


def parts_positions(text: str) -> dict[str, tuple[float, float]]:
    """MCU / JST / encoder centres in the stand frame (u, v)."""
    xs, ys = [], []
    for line in text.splitlines():
        if 'Edge.Cuts' in line and line.strip().startswith('(gr_'):
            for a, b in re.findall(
                r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line
            ):
                xs.append(float(a))
                ys.append(float(b))
    x_min, y_max = min(xs), max(ys)
    refs = {'MCU1': 'MCU/XIAO', 'JST1': 'JST', 'RE1': '旋钮'}
    out: dict[str, tuple[float, float]] = {}
    for chunk in text.split('(footprint ')[1:]:
        ref = re.search(r'\(property "Reference" "([^"]+)"', chunk)
        if not ref or ref.group(1) not in refs:
            continue
        at = re.search(r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)', chunk)
        x, y = float(at.group(1)), float(at.group(2))
        out[refs[ref.group(1)]] = (y_max - y, x - x_min)
    return out


def plate_outline(s: stand.Stand) -> list[tuple[float, float]]:
    """True backplate outline: mid-thickness section of plate_and_posts(),
    chained into an ordered polygon (plate frame x, y)."""
    body = s.plate_and_posts()
    slab = Pos(0, 0, stand.plate_t / 2) * Box(400, 400, 0.02)
    face = max((body & slab).faces(), key=lambda f: f.area)
    segs = []
    for e in face.outer_wire().edges():
        p = e @ 0
        q = e @ 1
        segs.append(((p.X, p.Y), (q.X, q.Y)))
    ring = [segs[0][0]]
    cur = segs[0][1]
    pool = segs[1:]
    while pool:
        i = min(
            range(len(pool)),
            key=lambda i: min(math.dist(cur, pool[i][0]),
                              math.dist(cur, pool[i][1])),
        )
        p, q = pool.pop(i)
        nxt = q if math.dist(cur, p) < math.dist(cur, q) else p
        cur = nxt
        if math.dist(cur, ring[0]) < 0.01:
            break
        ring.append(cur)
    return ring[:-1]


def main() -> None:
    args = parse_args()
    angles = [float(a) for a in args.set_angles.split(',')]
    with contextlib.redirect_stdout(io.StringIO()):
        pcb_w, pcb_h, holes, _outline = stand.parse_pcb(stand.DEFAULT_PCB)
        s = stand.Stand(pcb_w, pcb_h, holes, _outline, 3.0, 'nut', angles,
                        tilt=args.tilt)
    ang_, s_shift, lift, _corner_z = s.set_meta[0]
    text = stand.DEFAULT_PCB.read_text()
    ring = edge_ring(text)
    keys = switch_positions(text)
    parts = parts_positions(text)
    refs = sorted(stand.HOLES_EXPECTED)
    cu = sum(u for u, _ in holes) / len(holes)
    cv = sum(v for _, v in holes) / len(holes)
    r = math.radians(ang_)

    def to_plate(u: float, v: float) -> tuple[float, float]:
        ru = cu + (u - cu) * math.cos(r) - (v - cv) * math.sin(r)
        rv = cv + (u - cu) * math.sin(r) + (v - cv) * math.cos(r)
        return (ru - pcb_w / 2 + s_shift,
                rv + stand.bottom_ext - stand.keyboard_drop + lift)

    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(13.2, 8.2))

    # ---- panel A: PCB flat in the stand frame ------------------------------
    ax_a.add_patch(MplPolygon(ring, closed=True, facecolor=C_BOARD,
                              edgecolor=C_EDGE, lw=1.6, zorder=2))
    for u, v, _ref in keys.values():
        ax_a.add_patch(MplPolygon(
            [(u - 7.25, v - 7.25), (u + 7.25, v - 7.25),
             (u + 7.25, v + 7.25), (u - 7.25, v + 7.25)],
            closed=True, fill=False, edgecolor=C_KEY, lw=0.7, zorder=5))
    for name, (u, v) in parts.items():
        if name == 'MCU/XIAO':
            ax_a.add_patch(MplPolygon([(u - 10, v - 16), (u + 10, v - 16),
                                       (u + 10, v + 16), (u - 10, v + 16)],
                                      facecolor=C_MCU, edgecolor='k',
                                      zorder=5))
            ax_a.text(u, v, name, color='w', fontsize=9, ha='center',
                      va='center', zorder=6)
        elif name == '旋钮':
            ax_a.add_patch(plt.Circle((u, v), 11, facecolor='0.8',
                                      edgecolor='0.3', zorder=5))
            ax_a.text(u, v - 17, name, fontsize=9, ha='center', color='0.2')
        else:
            ax_a.add_patch(MplPolygon([(u - 6, v - 4), (u + 6, v - 4),
                                       (u + 6, v + 4), (u - 6, v + 4)],
                                      facecolor='0.75', edgecolor='0.3',
                                      zorder=5))
            ax_a.text(u, v + 8, name, fontsize=9, ha='center', color='0.2')
    hole_refs = {ref: (u, v) for ref, (u, v) in zip(refs, holes, strict=True)}
    for ref, (u, v) in hole_refs.items():
        ax_a.add_patch(plt.Circle((u, v), 5.5, facecolor='white',
                                  edgecolor=C_POST, lw=1.8, zorder=6))
        ax_a.text(u + 8, v + 6, ref, fontsize=9.5, color=C_POST,
                  fontweight='bold')
    u_thumb = min(u for name, (u, _v, _r) in keys.items()
                  if 'thumb' in name)
    ax_a.annotate('拇指簇朝上', xy=(u_thumb + 10, 150), xytext=(u_thumb + 6, 168),
                  fontsize=10.5, color=C_EDGE,
                  arrowprops={'arrowstyle': '-|>', 'color': C_EDGE, 'lw': 1.6})
    pk = keys['pinky_outer'][:2]
    ax_a.annotate('小指加高键（单键）朝下', xy=(pk[0] + 6, pk[1] - 9),
                  xytext=(pk[0] + 26, pk[1] - 26), fontsize=10.5,
                  color=C_EDGE,
                  arrowprops={'arrowstyle': '-|>', 'color': C_EDGE,
                              'lw': 1.6})
    ax_a.text(0.03, 0.02,
              '孔位映射（相对 KiCad 视图两轴翻转）:\n'
              'u = Y_max - Y_kicad（横向）\n'
              'v = X_kicad - X_min（高度）',
              transform=ax_a.transAxes, fontsize=9.5, va='bottom',
              bbox={'fc': 'white', 'ec': '0.7', 'alpha': 0.9})
    ax_a.set_aspect('equal')
    ax_a.set_xlim(-10, pcb_w + 10)
    ax_a.set_ylim(-30, pcb_h + 22)
    ax_a.set_xlabel('u（行方向）mm')
    ax_a.set_ylabel('v（列方向）mm')
    ax_a.set_title('PCB 站立系平放（未做平面内旋转）', fontsize=12)

    # ---- panel B: mounted on the backplate (plate frame) -------------------
    outline = plate_outline(s)
    ax_b.add_patch(MplPolygon(outline, closed=True, facecolor='0.92',
                              edgecolor='0.4', lw=1.4, zorder=1))
    board = [to_plate(u, v) for u, v in ring]
    ax_b.add_patch(MplPolygon(board, closed=True, facecolor=C_BOARD,
                              edgecolor=C_EDGE, lw=1.4, alpha=0.9, zorder=3))
    for u, v, _ref in keys.values():
        x, y = to_plate(u, v)
        ax_b.add_patch(MplPolygon(
            [(x - 7.25, y - 7.25), (x + 7.25, y - 7.25),
             (x + 7.25, y + 7.25), (x - 7.25, y + 7.25)],
            closed=True, fill=False, edgecolor=C_KEY, lw=0.6, zorder=5))
    for i, (x, y) in enumerate(s.all_posts):
        ref = refs[i]
        tapped = (x, y) == s.lowest_post
        ax_b.add_patch(plt.Circle((x, y), 7.5, facecolor='none',
                                  edgecolor=C_TAP if tapped else C_POST,
                                  lw=1.4, ls='--', zorder=6))
        ax_b.add_patch(plt.Circle((x, y), 3.8 if tapped else 4.5,
                                  facecolor='white',
                                  edgecolor=C_TAP if tapped else C_POST,
                                  lw=1.6, zorder=6))
        ax_b.text(x + 9.5, y + 6, ref + ('（自攻）' if tapped else ''),
                  fontsize=9.5, color=C_TAP if tapped else C_POST,
                  fontweight='bold', zorder=6)
    ax_b.text(0.03, 0.02,
              f'单组 4 柱位:图案绕自身质心旋转 {ang_:+g}°（home row 斜率），\n'
              '求解器横向居中并守住 PCB 最低点距底座顶 ≥2 mm，\n'
              f'整组再下移 {stand.keyboard_drop:g} mm；立板另后仰 '
              f'{s.tilt:g}°（此图未画）。\n'
              '虚线 = Ø15 柱喇叭口，H3 为 Ø3.8 自攻底孔（免螺母）。',
              transform=ax_b.transAxes, fontsize=9.5, va='bottom',
              bbox={'fc': 'white', 'ec': '0.7', 'alpha': 0.9})
    ax_b.set_aspect('equal')
    ax_b.set_xlim(-85, 90)
    ax_b.set_ylim(-35, 145)
    ax_b.set_xlabel('x（板横向）mm')
    ax_b.set_ylabel('y（板高）mm')
    ax_b.set_title(f'装上立板（板局部系，已旋转 {ang_:+g}°）', fontsize=12)

    fig.suptitle('C. Dux 安装朝向与单组柱位 —— '
                 f't{s.tilt:g} / set-angles {ang_:+g}°'
                 '（小指加高键朝下、拇指簇朝上）', fontsize=13.5, y=0.99)
    fig.savefig(args.out, dpi=160, bbox_inches='tight')
    print('saved', args.out)


if __name__ == '__main__':
    main()
