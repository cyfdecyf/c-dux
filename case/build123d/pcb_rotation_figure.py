#!/usr/bin/env python3
"""PCB rotation before/after comparison figure (t* / set-angles).

Four panels, every one of them showing the ground plane (hatched):
  left  top  — flat on the desk, stand frame top view (no in-plane rotation)
  left  side — flat side view, board back on the desk (0 deg)
  right top  — mounted, front view (in-plane rotation visible; the recline
               only foreshortens z by cos(tilt)); stand silhouette behind
               plus a zoom inset on the board corner next to the lowest key
  right side — mounted, side view: recline angle vs ground, board-to-boot
               gap

Inputs: geometry JSON written by keyboard_clearance_check.py --json, plus the
fused assembly STL exported by `stand.py --part full` (world pose).

Usage:
    uv run --with matplotlib --with trimesh --with shapely python \
        pcb_rotation_figure.py \
        --data out/keyboard_clearance.json \
        --stand-stl out/stand_left_full_t30.stl
"""

from __future__ import annotations

import argparse
import json
import math

from pathlib import Path

import matplotlib


matplotlib.use('Agg')
import matplotlib.pyplot as plt
import trimesh

from matplotlib.patches import Arc
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import MultiPoint
from shapely.geometry import Polygon as ShPolygon
from shapely.ops import unary_union


plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK HK',
                                   'WenQuanYi Zen Hei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

C_BOARD, C_EDGE = '#cfe3f7', '#2b6cb0'
C_RING, C_KEY, C_HL, C_CORNER = '#f59f00', '0.45', '#d6336c', '#e8590c'


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--data', default='out/keyboard_clearance.json',
                    help='JSON from keyboard_clearance_check.py --json')
    ap.add_argument('--stand-stl', default=None,
                    help='fused assembly STL (stand.py --part full); '
                    'default out/stand_left_full_t<tilt>.stl')
    ap.add_argument('--out', default='out/pcb_rotation_compare.png')
    ap.add_argument('--highlight-key', default='ring_TOP',
                    help='switch name (key of the JSON "keys" map) to mark')
    ap.add_argument('--highlight-label', default='无名指列 top',
                    help='display name for the highlighted switch')
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    figd = json.loads(Path(args.data).read_text())
    ct, st = figd['ct'], figd['st']
    face_z, origin_z, pcb_w = figd['face_z'], figd['origin_z'], figd['pcb_w']
    ang = figd['set_angle']
    drop, bot_ext = 30.0, 6.5  # stand.keyboard_drop / bottom_ext
    ring = [tuple(p) for p in figd['ring']]
    keys = figd['keys']
    holes = figd['holes']
    cu = sum(p[0] for p in holes.values()) / len(holes)
    cv = sum(p[1] for p in holes.values()) / len(holes)
    r = math.radians(ang)
    stl = args.stand_stl or f'out/stand_left_full_t{figd["tilt"]:g}.stl'

    def to_plate_uv(u: float, v: float) -> tuple[float, float]:
        ru = cu + (u - cu) * math.cos(r) - (v - cv) * math.sin(r)
        rv = cv + (u - cu) * math.sin(r) + (v - cv) * math.cos(r)
        return ru - pcb_w / 2 + figd['shift'], rv + bot_ext - drop + figd['lift']

    def world(
        x_l: float, y_l: float, z_l: float,
    ) -> tuple[float, float, float]:
        return (x_l, st * y_l - ct * z_l,
                origin_z + ct * y_l + st * z_l)

    plate = {k: to_plate_uv(*d['uv']) for k, d in keys.items()}
    hl = args.highlight_key
    hl_w = world(*plate[hl], face_z)

    # corner next to the highlighted/lowest switch: precomputed in the JSON
    cor = figd['ring_top_corner']
    vi, vc = cor['idx'], tuple(cor['uv'])
    cw = tuple(cor['world'])
    prev_v, next_v, next2_v = (
        ring[(vi + off) % len(ring)] for off in (-1, 1, 2))
    # fallback if the JSON predates the corner block
    if not cw:
        zs = [world(*to_plate_uv(u, v), face_z)[2] for u, v in ring]
        vi = min(range(len(ring)), key=lambda i: zs[i])
        vc = ring[vi]
        cw = world(*to_plate_uv(*vc), face_z)

    def front_deg(p_from: tuple[float, float],
                  p_to: tuple[float, float]) -> float:
        du, dv = p_to[0] - p_from[0], p_to[1] - p_from[1]
        dpx = du * math.cos(r) - dv * math.sin(r)
        dpy = du * math.sin(r) + dv * math.cos(r)
        return math.degrees(math.atan2(ct * dpy, dpx))

    def flat_deg(p_from: tuple[float, float],
                 p_to: tuple[float, float]) -> float:
        return math.degrees(
            math.atan2(p_to[1] - p_from[1], p_to[0] - p_from[0]))

    def tip_deg(p_corner: tuple[float, float], proj: str) -> float:
        us = []
        for p in (prev_v, next_v):
            du, dv = p[0] - p_corner[0], p[1] - p_corner[1]
            if proj == 'flat':
                us.append(math.atan2(dv, du))
            else:
                dpx = du * math.cos(r) - dv * math.sin(r)
                dpy = du * math.sin(r) + dv * math.cos(r)
                us.append(math.atan2(ct * dpy, dpx))
        bx, by = sum(math.cos(a) for a in us), sum(math.sin(a) for a in us)
        return math.degrees(math.atan2(-by, -bx))

    tip_flat, tip_front = tip_deg(vc, 'flat'), tip_deg(vc, 'front')
    a_prev, a_next = front_deg(vc, prev_v), front_deg(vc, next_v)
    a_edge = front_deg(next_v, next2_v)
    f_prev, f_next = flat_deg(vc, prev_v), flat_deg(vc, next_v)
    print(f'corner: flat tip {tip_flat:.0f} deg (edges {f_prev:.0f}/'
          f'{f_next:.0f}); front tip {tip_front:.0f} deg '
          f'(edges {a_prev:.0f}/{a_next:.0f}, right edge {a_edge:.0f})')

    mesh = trimesh.load(stl)
    v_arr, faces = mesh.vertices, mesh.faces

    def silhouette(ix: int, iz: int):
        polys = [ShPolygon([(v_arr[f[0], ix], v_arr[f[0], iz]),
                            (v_arr[f[1], ix], v_arr[f[1], iz]),
                            (v_arr[f[2], ix], v_arr[f[2], iz])])
                 for f in faces]
        return unary_union(polys).buffer(0)

    sil_front, sil_side = silhouette(0, 2), silhouette(1, 2)

    def draw_silhouette(ax, sil):
        geoms = sil.geoms if sil.geom_type == 'MultiPolygon' else [sil]
        for poly in geoms:
            ax.add_patch(MplPolygon(poly.exterior.coords, closed=True,
                                    facecolor='0.88', edgecolor='0.55',
                                    lw=0.8, zorder=1))

    def draw_ground(ax, depth: float, *, label: bool = True) -> None:
        ax.axhline(0, color='k', lw=2)
        ax.axhspan(-depth, 0, facecolor='0.96', hatch='///',
                   edgecolor='0.55', lw=0, zorder=0)
        if label:
            ax.text(0.012, 0.05, '水平面（桌面）', transform=ax.transAxes,
                    fontsize=9.5, va='bottom', style='italic', color='0.25')

    def boxed(ax, x, y, s: str, color: str, **kw) -> None:
        ax.text(x, y, s, fontsize=8.5, color=color, zorder=7,
                bbox={'fc': 'white', 'ec': 'none', 'alpha': 0.75, 'pad': 0.4}, **kw)

    fig = plt.figure(figsize=(13.5, 9.4))
    gs = fig.add_gridspec(2, 2, height_ratios=[3.1, 1.0], hspace=0.18,
                          wspace=0.10)

    # ==== left top: flat top view ==========================================
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.add_patch(MplPolygon(ring, closed=True, facecolor=C_BOARD,
                             edgecolor=C_EDGE, lw=1.6, zorder=2))
    for k, d in keys.items():
        u, v = d['uv']
        col = C_RING if k.startswith('ring') else C_KEY
        for half, ls in ((9.0, ':'), (7.25, '-')):
            cs = [(u - half, v - half), (u + half, v - half),
                  (u + half, v + half), (u - half, v + half)]
            ax1.add_patch(MplPolygon(cs, closed=True, fill=False,
                                     edgecolor=col,
                                     lw=1.2 if col == C_RING else 0.7,
                                     ls=ls, zorder=5))
    ax1.plot(*keys[hl]['uv'], 'o', color=C_HL, ms=8, zorder=6)
    ax1.annotate(f'{args.highlight_label}\n（平放系键位）',
                 xy=keys[hl]['uv'],
                 xytext=(keys[hl]['uv'][0] - 60,
                         keys[hl]['uv'][1] + 20),
                 fontsize=10, color=C_HL,
                 arrowprops={'arrowstyle': '->', 'color': C_HL})
    ax1.plot(*vc, 'D', color=C_CORNER, ms=7, zorder=6)
    ax1.annotate(f'PCB 角（{hl} 旁，全局最低板料点）\n'
                 f'平放时角尖朝向 ≈{tip_flat:.0f}°\n'
                 f'（两条边方向 {f_prev:.0f}° / +{f_next:.0f}°）',
                 xy=vc, xytext=(vc[0] - 74, vc[1] - 46), fontsize=10,
                 color=C_CORNER,
                 arrowprops={'arrowstyle': '->', 'color': C_CORNER})
    ax1.plot(cu, cv, '+', color='k', ms=10, zorder=6)
    ax1.annotate('安装图案质心\n整板绕它旋转 '
                 f'{ang:+g}°', xy=(cu, cv),
                 xytext=(cu - 30, cv - 58), fontsize=9.5,
                 arrowprops={'arrowstyle': '->', 'lw': 0.9})
    ax1.annotate('', xy=(cu + 32, cv + 4), xytext=(cu + 24, cv + 20),
                 arrowprops={'arrowstyle': '-|>', 'color': 'k', 'lw': 1.4,
                                 'connectionstyle': 'arc3,rad=-0.5'})
    ax1.text(0.985, 0.975,
             '旋转前：平放俯视\n（stand 坐标系 = ergogen 视图转 90°，\n'
             '小指列朝下；纸面即水平面，板背贴桌面）',
             transform=ax1.transAxes, ha='right', va='top', fontsize=10,
             bbox={'fc': 'white', 'ec': '0.7', 'alpha': 0.9})
    ax1.set_aspect('equal')
    ax1.set_xlim(-8, 122)
    ax1.set_ylim(-30, 162)
    ax1.set_xlabel('u（行方向）mm')
    ax1.set_ylabel('v（列方向）mm')
    ax1.set_title('旋转前 —— 平放俯视（未做平面内旋转）', fontsize=12)

    # ==== left bottom: flat side view ======================================
    ax2 = fig.add_subplot(gs[1, 0])
    draw_ground(ax2, 6)
    ax2.add_patch(MplPolygon([(0, 0), (figd['pcb_h'], 0),
                              (figd['pcb_h'], 1.6), (0, 1.6)],
                             facecolor=C_BOARD, edgecolor=C_EDGE, lw=1.2))
    for k, d in keys.items():
        v = d['uv'][1]
        col = C_RING if k.startswith('ring') else C_KEY
        ax2.add_patch(MplPolygon([(v - 7.25, 1.6), (v + 7.25, 1.6),
                                  (v + 7.25, 9.1), (v - 7.25, 9.1)],
                                 fill=False, edgecolor=col, lw=0.8))
        ax2.add_patch(MplPolygon([(v - 9, 6.1), (v + 9, 6.1),
                                  (v + 9, 11.6), (v - 9, 11.6)],
                                 fill=False, edgecolor=col, lw=0.7, ls=':'))
    ax2.plot(keys[hl]['uv'][1], face_z, 'o', color=C_HL, ms=7)
    ax2.set_xlim(-10, figd['pcb_h'] + 10)
    ax2.set_ylim(-9, 24)
    ax2.set_aspect('equal')
    ax2.set_xlabel('v（列方向）mm')
    ax2.set_title('旋转前侧视：板背平贴地面（0°）', fontsize=11)

    # ==== right top: mounted front view ====================================
    ax3 = fig.add_subplot(gs[0, 1])
    draw_ground(ax3, 26)
    draw_silhouette(ax3, sil_front)
    ax3.text(46, 3.4, '底座', fontsize=10, color='0.35', ha='center')
    ax3.text(24, 10, '靴座', fontsize=10, color='0.25', ha='center')

    def proj_front(x_l: float, y_l: float, z_l: float) -> tuple[float, float]:
        w = world(x_l, y_l, z_l)
        return (w[0], w[2])

    back_poly = [proj_front(*to_plate_uv(u, v), face_z) for u, v in ring]
    front_poly = [proj_front(*to_plate_uv(u, v), face_z + 1.6)
                  for u, v in ring]
    ax3.add_patch(MplPolygon(front_poly[::-1] + back_poly, closed=True,
                             facecolor=C_BOARD, edgecolor=C_EDGE, lw=1.6,
                             alpha=0.95, zorder=3))
    for k, d in keys.items():
        ring_col = k.startswith('ring')
        u, v = d['uv']
        for half, cap_off in ((9.0, 3.5), (7.25, 0.0)):
            cs = [(u - half, v - half), (u + half, v - half),
                  (u + half, v + half), (u - half, v + half)]
            pts = [proj_front(*to_plate_uv(cx, cy), face_z + 1.6 + cap_off)
                   for cx, cy in cs]
            ax3.add_patch(MplPolygon(
                pts, closed=True, fill=False,
                edgecolor=(C_RING if ring_col else C_KEY),
                lw=1.2 if ring_col else 0.7,
                ls=':' if half == 9.0 else '-', zorder=5))
    ax3.plot(hl_w[0], hl_w[2], 'o', color=C_HL, ms=8, zorder=6)
    ax3.annotate(f'{args.highlight_label}\n安装后最低键  z='
                 f'{figd["switch_z"][0][0]:.1f}',
                 xy=(hl_w[0], hl_w[2]),
                 xytext=(hl_w[0] + 24, hl_w[2] + 30), fontsize=10,
                 color=C_HL, zorder=7,
                 bbox={'fc': 'white', 'ec': 'none', 'alpha': 0.75, 'pad': 0.5},
                 arrowprops={'arrowstyle': '->', 'color': C_HL})
    ax3.plot(cw[0], cw[2], 'D', color=C_CORNER, ms=7, zorder=6)
    ax3.annotate(f'PCB 角（{hl} 旁）z≈{cw[2]:.1f}\n'
                 f'正视图角尖朝向 ≈{tip_front:.0f}°\n'
                 '底座顶上方 3.8 mm、靴座前 3 mm',
                 xy=(cw[0], cw[2]), xytext=(cw[0] - 78, cw[2] + 24),
                 fontsize=10, color=C_CORNER, zorder=7,
                 bbox={'fc': 'white', 'ec': 'none', 'alpha': 0.75, 'pad': 0.5},
                 arrowprops={'arrowstyle': '->', 'color': C_CORNER})
    ax3.axhline(origin_z, ls='--', lw=0.8, color='0.5')
    ax3.text(-76, origin_z + 1.4, '座面 z=6', fontsize=8.5, color='0.4')
    ax3.text(0.985, 0.975,
             f'旋转后：t{figd["tilt"]:g} / {ang:+g}° 正视图\n'
             '（平面内旋转可见；后仰角\n只表现为竖向按 cos(tilt) 压缩，见右下）',
             transform=ax3.transAxes, ha='right', va='top', fontsize=10,
             bbox={'fc': 'white', 'ec': '0.7', 'alpha': 0.9})
    ax3.set_aspect('equal')
    ax3.set_xlim(-80, 86)
    ax3.set_ylim(-6, 152)
    ax3.set_xlabel('x（横向）mm')
    ax3.set_ylabel('离地高 z mm')
    ax3.set_title(f'旋转后 —— t{figd["tilt"]:g} / set-angles {ang:+g}°'
                  '（正视图）', fontsize=12)

    # inset zoom near the corner
    axins = ax3.inset_axes([0.025, 0.50, 0.42, 0.46])
    axins.set_facecolor('white')
    axins.set_xlim(cw[0] - 24, cw[0] + 17)
    axins.set_ylim(cw[2] - 8.5, cw[2] + 12.5)
    axins.axhline(0, color='k', lw=1.6)
    axins.axhspan(-8, 0, facecolor='0.96', hatch='///', edgecolor='0.55',
                  lw=0)
    axins.axhline(6, color='0.35', lw=1.2, ls='--', zorder=2)
    boxed(axins, cw[0] - 23, 6.4, '底座顶 z=6', '0.3')
    boot_f, ledge = figd['boot']['front'], figd['boot']['top']
    axins.plot([boot_f, boot_f + 8 * st / ct], [6, ledge], color='0.25',
               lw=2.2, zorder=2)
    axins.plot([boot_f + 8 * st / ct, cw[0] + 16], [ledge, ledge],
               color='0.25', lw=2.2, zorder=2)
    boxed(axins, boot_f - 1.2, 12.3, '靴座前面', '0.15', ha='right')
    boxed(axins, cw[0] + 16, ledge - 1.6, '靴座顶 z=14', '0.15', ha='right')
    axins.add_patch(MplPolygon(back_poly, closed=True, facecolor=C_BOARD,
                               edgecolor=C_EDGE, lw=1.4, alpha=0.55,
                               zorder=4))
    axins.plot(cw[0], cw[2], 'D', color=C_CORNER, ms=6, zorder=6)
    axins.annotate('', xy=(cw[0] + 11 * math.cos(math.radians(tip_front)),
                           cw[2] + 11 * math.sin(math.radians(tip_front))),
                   xytext=(cw[0], cw[2]),
                   arrowprops={'arrowstyle': '-|>', 'color': C_CORNER, 'lw': 1.3})
    boxed(axins, cw[0] - 13.5, cw[2] - 6.2, f'角尖朝向 ≈{tip_front:.0f}°',
          C_CORNER, ha='center')
    for p_to, lt in ((prev_v, (cw[0] - 12.0, cw[2] + 10.8)),
                     (next_v, (cw[0] + 12.6, cw[2] + 4.6))):
        a = math.radians(front_deg(vc, p_to))
        axins.annotate(
            '', xy=(cw[0] + 14 * math.cos(a), cw[2] + 14 * math.sin(a)),
            xytext=(cw[0], cw[2]),
            arrowprops={'arrowstyle': '-|>', 'color': C_CORNER, 'lw': 1.0})
        boxed(axins, *lt, f'{front_deg(vc, p_to):+.0f}°', C_CORNER,
              ha='center', va='center')
    aa = math.radians(a_edge)
    x0, z0 = proj_front(*to_plate_uv(*next_v), face_z)
    axins.annotate(
        '', xy=(x0 + 10 * math.cos(aa), z0 + 10 * math.sin(aa)),
        xytext=(x0, z0),
        arrowprops={'arrowstyle': '-|>', 'color': C_CORNER, 'lw': 1.0})
    boxed(axins, x0 + 12.5 * math.cos(aa), z0 + 12.5 * math.sin(aa) + 1.2,
          f'{a_edge:+.0f}°', C_CORNER, ha='center')
    axins.annotate('', xy=(cw[0] + 5.5, 6), xytext=(cw[0] + 5.5, cw[2]),
                   arrowprops={'arrowstyle': '<->', 'color': 'k', 'lw': 1.0})
    boxed(axins, cw[0] + 6.2, (6 + cw[2]) / 2,
          f'距底座顶\n{cw[2] - 6:.1f} mm', 'k', va='center')
    axins.plot(hl_w[0], hl_w[2], 'o', color=C_HL, ms=5, zorder=6)
    axins.annotate(args.highlight_label.split('（')[0],
                   xy=(hl_w[0], hl_w[2]),
                   xytext=(hl_w[0] - 4.5, hl_w[2] - 4.5), fontsize=8.5,
                   color=C_HL, zorder=7,
                   arrowprops={'arrowstyle': '->', 'color': C_HL, 'lw': 0.9})
    axins.tick_params(labelsize=7)
    axins.set_title('角点放大（正视图）', fontsize=9)
    ax3.indicate_inset_zoom(axins, edgecolor='0.5')

    # ==== right bottom: mounted side view ==================================
    ax4 = fig.add_subplot(gs[1, 1])
    draw_ground(ax4, 7)
    draw_silhouette(ax4, sil_side)
    pcb_pts = [world(*to_plate_uv(u, v), face_z)[1:] for u, v in ring]
    pcb_pts += [world(*to_plate_uv(u, v), face_z + 1.6)[1:] for u, v in ring]
    hull = MultiPoint(pcb_pts).convex_hull
    ax4.add_patch(MplPolygon(hull.exterior.coords, closed=True,
                             facecolor=C_BOARD, edgecolor=C_EDGE, lw=1.4,
                             alpha=0.95))
    for k, p in plate.items():
        w = world(*p, face_z + 1.6 + 3.5)
        col = C_HL if k == hl else (C_RING if k.startswith('ring') else C_KEY)
        ax4.plot(w[1], w[2], 'o', color=col,
                 ms=6 if k == hl else 3.5, zorder=6)
    w0 = world(0, 0, face_z)
    w1 = world(0, 60, face_z)
    ax4.plot([w0[1], w1[1]], [w0[2], w1[2]], color=C_EDGE, lw=1.0)
    ax4.plot([w0[1], w0[1] + 34], [w0[2], w0[2]], color='0.3', lw=0.9,
             ls='--')
    ax4.add_patch(Arc((w0[1], w0[2]), 30, 30, theta1=0,
                      theta2=math.degrees(math.atan2(ct, st)), color='0.2',
                      lw=1.2))
    board_ground = math.degrees(math.atan2(ct, st))  # board vs ground
    recline = 90 - board_ground
    ax4.text(w0[1] + 19, w0[2] + 5,
             f'板面与地面 {board_ground:.0f}°\n（立板后仰 {recline:.0f}°）',
             fontsize=9.5, color='0.1')
    ga = world(*to_plate_uv(*vc), face_z)
    gb = world(*to_plate_uv(*vc), 6.0)
    ax4.annotate('', xy=gb[1:], xytext=ga[1:],
                 arrowprops={'arrowstyle': '<->', 'color': 'k', 'lw': 1.0})
    ax4.text((ga[1] + gb[1]) / 2 + 2.5, (ga[2] + gb[2]) / 2,
             '板背面—靴座前面 3 mm', fontsize=8.5)
    ax4.set_aspect('equal')
    ax4.set_xlim(-32, 92)
    ax4.set_ylim(-8, 55)
    ax4.set_xlabel('y（进深，左=朝用户）mm')
    ax4.set_title(f'旋转后侧视：板面与地面 {board_ground:.0f}°'
                  f'（红点 = {hl}）', fontsize=11)

    fig.suptitle(f'C. Dux PCB 旋转前后对比 —— t{figd["tilt"]:g} / '
                 f'set-angles {ang:+g}°'
                 '（左列平放，右列装上立板；地面 = 阴影线）',
                 fontsize=13.5, y=0.985)
    fig.savefig(args.out, dpi=160, bbox_inches='tight')
    print('saved', args.out)


if __name__ == '__main__':
    main()
