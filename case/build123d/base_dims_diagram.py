#!/usr/bin/env python3
"""Annotated dimensions figure for the C. Dux stand base (README image).

Top view: base_front / base_rear zones, depth, pads (= the tip-over
pivots), shear keys, joint screws, assembly CoM and the horizontal strike
component. Side view: the W*d >= F*h tip model with the force split and
the user's sight line -- in standing use (handshake grip, see
images/c-dux-v2-in-use.jpg) this profile is what the user faces.

Every drawn dimension is pulled from stand.py, so the figure cannot drift
from the model: change a constant there, rerun this, and the figure
follows. Writes images/stand-base-dimensions.{svg,png} (embedded in
README.md); the PNG needs rsvg-convert on PATH, the SVG is always written.

Usage:
    uv run base_dims_diagram.py
"""

from __future__ import annotations

import math
import shutil
import subprocess

from pathlib import Path

import stand as st


REPO = Path(__file__).resolve().parents[2]
OUT_DIR = REPO / 'images'
STEM = 'stand-base-dimensions'

POST_H = 5.0   # mirror stand.py's --post-h default (PCB-to-plate gap)

# canvas layout, px (fits the current geometry with headroom for labels)
S_TOP, S_SIDE = 3.0, 2.0      # px per mm
W_TOP, H_TOP = 850, 520
W_SIDE, H_SIDE = 580, 500
O_TOP = (300.0, 40.0)         # top-view offset: px of the world origin corner
O_SIDE = (60.0, 70.0)
Z_VIEW = 175.0                # side-view world height, mm

RHO = 1.27     # g/cm3, solid PETG -- only ratios matter for the CoM dot
M_KB = 150.0   # g, assumed keyboard half mass (PCB + switches + caps + MCU)
KB_TOFF = 7.0  # mm, cap/switch centroid in front of the plate face

HALO = 'stroke="#fff" stroke-width="4" paint-order="stroke"'


def geometry() -> dict:
    """Every drawn number, derived from stand.py (and the v2.0 PCB)."""
    pcb_w, pcb_h, holes = st.parse_pcb(st.DEFAULT_PCB)
    s = st.Stand(pcb_w, pcb_h, holes, POST_H, 'nut', st.SET_ANGLES, st.TILT)

    tilt = math.radians(s.tilt)
    sn, ct = math.sin(tilt), math.cos(tilt)
    pads = s.pad_positions()
    kb_t0 = POST_H + 1.5   # PCB face offset from the plate (posts + board)
    kb_t1 = kb_t0 + 8.5    # + keycap stack

    vb, vu = s.base_solid(), s.upright_solid()
    m_b, c_b = vb.volume * RHO / 1000, vb.center()
    m_u, c_u = vu.volume * RHO / 1000, vu.center()
    kb_y = pcb_h / 2 * sn - KB_TOFF * ct
    kb_z = s.origin_z + pcb_h / 2 * ct + KB_TOFF * sn
    m_tot = m_b + m_u + M_KB
    com_y = (m_b * c_b.Y + m_u * c_u.Y + M_KB * kb_y) / m_tot
    com_z = (m_b * c_b.Z + m_u * c_u.Z + M_KB * kb_z) / m_tot

    return {
        'bx': s.bx, 'y_front': s.y0, 'y_rear': s.y1,
        'pads': pads, 'pad_d': st.pad_d,
        'pad_f': min(y for _, y in pads), 'pad_r': max(y for _, y in pads),
        'key_x': st.key_x, 'key_l': st.key_l,
        'key_y0': st.key_y0, 'key_y1': st.key_y1,
        'js_x': st.joint_screw_x, 'js_y': st.joint_screw_y,
        'pocket_r': st.head_pocket_d / 2, 'bore_r': st.joint_screw_d / 2,
        'tilt': s.tilt, 'sn': sn, 'ct': ct,
        'plate_w': s.plate_w, 'plate_h': s.plate_h, 'plate_t': st.plate_t,
        'kb_t0': kb_t0, 'kb_t1': kb_t1,
        'origin_z': s.origin_z, 'com_y': com_y, 'com_z': com_z,
        'f_z': round(s.origin_z + s.plate_h * ct + kb_t1 * sn),
    }


# ---------------------------------------------------------------- svg bits

def defs() -> str:
    """Arrowhead markers: black, red (tipping) and green (preloading)."""
    marks = {'ah': '#333', 'ahR': '#d62728', 'ahG': '#1a7f37'}
    body = ''.join(
        f'<marker id="{i}" viewBox="0 0 10 10" refX="9" refY="5" '
        f'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
        f'<path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
        for i, c in marks.items())
    return f'<defs>{body}</defs>'


def txt(x, y, body, size=14, fill='#222', *, anchor='', halo=False,
        rot=None) -> str:
    attrs = f'x="{x}" y="{y}" font-size="{size}" fill="{fill}"'
    attrs += ' font-weight="bold"'
    if anchor:
        attrs += f' text-anchor="{anchor}"'
    if halo:
        attrs += f' {HALO}'
    if rot is not None:
        attrs += f' transform="rotate({rot} {x} {y})"'
    return f'<text {attrs}>{body}</text>'


def ln(x1, y1, x2, y2, stroke='#333', w=1.6, dash='', marker=''):
    a = f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{stroke}"'
    a += f' stroke-width="{w}"'
    if dash:
        a += f' stroke-dasharray="{dash}"'
    if marker:
        a += f' marker-start="url(#{marker})" marker-end="url(#{marker})"'
    return f'<line {a}/>'


def dim_v(x, y1, y2, label, color='#333') -> str:
    """Vertical dimension line with a rotated label."""
    mid = (y1 + y2) / 2
    line = ln(x, y1, x, y2, color, 1.6, marker='ah')
    text = (f'<text x="{x - 9}" y="{mid}" font-size="15" fill="{color}" '
            f'font-weight="bold" text-anchor="middle" '
            f'transform="rotate(-90 {x - 9} {mid})">{label}</text>')
    return line + text


# ---------------------------------------------------------------- panels

def top_view(g: dict) -> str:
    bx, y_front, y_rear = g['bx'], g['y_front'], g['y_rear']
    ox, oy = O_TOP

    def t(x, y):
        return (ox + x * S_TOP, oy + (y_rear - y) * S_TOP)

    e: list[str] = []
    bl, bt = t(-bx / 2, y_rear)
    bh, bw = (y_rear - y_front) * S_TOP, bx * S_TOP
    e.append(f'<clipPath id="board"><rect x="{bl}" y="{bt}" width="{bw}" '
             f'height="{bh}" rx="{10 * S_TOP}"/></clipPath>')
    e.append(f'<rect x="{bl}" y="{bt}" width="{bw}" height="{bh}" '
             f'rx="{10 * S_TOP}" fill="#fff" stroke="#222" stroke-width="2.5"/>')
    zx, zy = t(-bx / 2, 0)
    e.append(f'<g clip-path="url(#board)">'
             f'<rect x="{bl}" y="{zy}" width="{bw}" height="{zy - bt}" '
             f'fill="#3498db" opacity="0.10"/>'
             f'<rect x="{bl}" y="{zy}" width="{bw}" '
             f'height="{t(-bx / 2, y_front)[1] - zy}" '
             f'fill="#e74c3c" opacity="0.10"/></g>')

    # keyboard + upright plan projection (board leans back, plate flush)
    top_y = g['plate_h'] * g['sn']
    kx, ky = t(-g['plate_w'] / 2, top_y)
    kh = (top_y + g['kb_t1'] * g['ct']) * S_TOP
    e.append(f'<rect x="{kx}" y="{ky}" width="{g["plate_w"] * S_TOP}" '
             f'height="{kh}" fill="#6a1b9a" fill-opacity="0.22" '
             f'stroke="#6a1b9a" stroke-dasharray="6 4" stroke-width="1.6"/>')
    p = t(0, 44)
    e.append(txt(p[0], p[1], f'键盘+立板投影（{g["tilt"]:g}°后仰）', 15,
                 '#6a1b9a', anchor='middle', halo=True))

    # pad contact lines = the tip-over pivots
    for yy, lbl, c in [(g['pad_f'], '前垫线 = 前倾支点', '#777'),
                       (g['pad_r'], '后垫线 = 后倾支点 ★', '#c0392b')]:
        a, b = t(-bx / 2 - 25, yy), t(bx / 2 + 4, yy)
        e.append(ln(a[0], a[1], b[0], b[1], c, 1.8, '8 5'))
        e.append(txt(6, a[1] - 3, lbl, 14, c))
    for px_, py_ in g['pads']:
        c = t(px_, py_)
        e.append(f'<circle cx="{c[0]}" cy="{c[1]}" '
                 f'r="{g["pad_d"] / 2 * S_TOP}" fill="#555" '
                 f'fill-opacity="0.9"/>')

    # joint: shear keys + M4 screws (with head-pocket circles)
    for sx in (-1, 1):
        a = t(sx * g['key_x'] - g['key_l'] / 2, g['key_y1'])
        b = t(sx * g['key_x'] + g['key_l'] / 2, g['key_y0'])
        e.append(f'<rect x="{a[0]}" y="{a[1]}" width="{b[0] - a[0]}" '
                 f'height="{b[1] - a[1]}" fill="#fff" stroke="#222" '
                 f'stroke-width="1.5"/>')
        c = t(sx * g['js_x'], g['js_y'])
        e.append(f'<circle cx="{c[0]}" cy="{c[1]}" '
                 f'r="{g["bore_r"] * S_TOP}" fill="#222"/>')
        e.append(f'<circle cx="{c[0]}" cy="{c[1]}" '
                 f'r="{g["pocket_r"] * S_TOP}" fill="none" stroke="#222" '
                 f'stroke-dasharray="3 3"/>')
    p = t(g['key_x'] + g['key_l'] / 2 + 4, 21)
    e.append(txt(p[0], p[1], '榫键×2 / M4×2（底座↔立板接头）',  # noqa: RUF001
                 13, halo=True))

    # seat origin
    a, b = t(-bx / 2, 0), t(bx / 2, 0)
    e.append(ln(a[0], a[1], b[0], b[1], '#222', 2, '2 4'))
    e.append(ln(a[0] + 3, a[1] + 1, 150, 462, '#222', 0.8, '3 3'))
    e.append(txt(156, 466, 'y=0 座面原点（立板/键盘安装参考线）', 14))

    # CoM + horizontal strike component
    c = t(0, g['com_y'])
    e.append(f'<circle cx="{c[0]}" cy="{c[1]}" r="5" fill="none" '
             f'stroke="#000" stroke-width="2"/>'
             f'<line x1="{c[0] - 8}" y1="{c[1]}" x2="{c[0] + 8}" y2="{c[1]}" '
             f'stroke="#000" stroke-width="2"/>'
             f'<line x1="{c[0]}" y1="{c[1] - 8}" x2="{c[0]}" '
             f'y2="{c[1] + 8}" stroke="#000" stroke-width="2"/>')
    p = t(-48, 21)
    e.append(txt(p[0], p[1], f'组装重心 y≈{g["com_y"]:.0f}', 14, halo=True))
    a, b = t(-30, 30), t(-30, 82)
    e.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" '
             f'stroke="#d62728" stroke-width="5" marker-end="url(#ahR)"/>')
    p = t(-22, 74)
    e.append(txt(p[0], p[1], '敲击力水平分量（推向后仰侧）', 16, '#d62728',
                 halo=True))

    # dimensions
    xa = t(bx / 2 + 14, 0)[0]
    xa2 = xa + 95
    xd = t(-bx / 2 - 15, 0)[0]
    for yy, off in [(0, 14), (y_front, 14), (y_rear, 109)]:
        p1, p2 = t(bx / 2, yy), t(bx / 2 + off, yy)
        e.append(ln(p1[0], p1[1], p2[0], p2[1], '#333', 0.8, '3 3'))
    for yy in (y_front, y_rear):
        p1, p2 = t(-bx / 2, yy), t(-bx / 2 - 15, yy)
        e.append(ln(p1[0], p1[1], p2[0], p2[1], '#333', 0.8, '3 3'))
    e.append(dim_v(xa, t(0, y_front)[1], t(0, 0)[1],
                   f'base_front = {-y_front:g}'))
    e.append(dim_v(xa2, t(0, y_rear)[1], t(0, 0)[1],
                   f'base_rear = {y_rear:g}'))
    e.append(dim_v(xd, t(0, y_front)[1], t(0, y_rear)[1],
                   f'进深 = front + rear = {y_rear - y_front:g}'))
    e.append(txt(W_TOP / 2, H_TOP - 26,
                 '图下方 = 前方（板面朝向）　图上方 = 后方（后仰侧）　·　'
                 '实际使用：站立，板面平行于身体，站位在图左/右侧', 15,
                 anchor='middle'))
    e.append(txt(W_TOP / 2, H_TOP - 6,
                 '握手位敲击：指力向下（经柱直传底座，压紧）+ '
                 '水平向后（倾覆，见右图）', 15, anchor='middle'))
    return (f'<svg x="0" y="0" width="{W_TOP}" height="{H_TOP}">'
            + ''.join(e) + '</svg>')


def side_view(g: dict) -> str:
    ox, oy = O_SIDE

    def u(y, z):
        return (ox + (y + 48) * S_SIDE, oy + (Z_VIEW - z) * S_SIDE)

    e: list[str] = []
    a, b = u(-48, 0), u(118, 0)
    e.append(ln(a[0], a[1], b[0], b[1], '#222', 2.5))
    for i in range(14):
        x = a[0] + i * 22
        e.append(f'<line x1="{x}" y1="{a[1]}" x2="{x - 10}" '
                 f'y2="{a[1] + 10}" stroke="#999" stroke-width="1.2"/>')
    for py in (g['pad_f'], g['pad_r']):
        p1, p2 = u(py - g['pad_d'] / 2, 0), u(py + g['pad_d'] / 2, 2.5)
        e.append(f'<rect x="{p1[0]}" y="{p1[1]}" width="{p2[0] - p1[0]}" '
                 f'height="{p2[1] - p1[1]}" rx="2" fill="#444"/>')

    # upright + keyboard bars, reclined by the stand tilt, seated at y=0
    tilt = math.radians(g['tilt'])
    sn, ct = math.sin(tilt), math.cos(tilt)

    def q(o, v):
        return u(v * sn - o * ct, g['origin_z'] + v * ct + o * sn)

    def bar(off0, off1, v0, v1, fill, stroke):
        pts = ' '.join(f'{p[0]:.1f},{p[1]:.1f}'
                       for p in (q(off0, v0), q(off1, v0), q(off1, v1),
                                 q(off0, v1)))
        return (f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" '
                f'stroke-width="1.8"/>')
    e.append(bar(g['kb_t0'], g['kb_t1'], 0, g['plate_h'],
                 '#f5efc9', '#8a7d3a'))
    e.append(bar(-g['plate_t'], 0, 0.5, g['plate_h'], '#d7bde2', '#6a1b9a'))

    # base slab with front/rear zone tints
    slab_top = g['origin_z'] + 2
    b1, b2 = u(g['y_front'], slab_top), u(g['y_rear'], 2.5)
    z1, z2 = u(0, slab_top), u(g['y_rear'], 2.5)
    zf1, zf2 = u(g['y_front'], slab_top), u(0, 2.5)
    e.append(f'<rect x="{b1[0]}" y="{b1[1]}" width="{b2[0] - b1[0]}" '
             f'height="{b2[1] - b1[1]}" fill="#eee" stroke="#222" '
             f'stroke-width="2"/>')
    e.append(f'<rect x="{z1[0]}" y="{z1[1]}" width="{z2[0] - z1[0]}" '
             f'height="{z2[1] - z1[1]}" fill="#3498db" opacity="0.10"/>'
             f'<rect x="{zf1[0]}" y="{zf1[1]}" width="{zf2[0] - zf1[0]}" '
             f'height="{zf2[1] - zf1[1]}" fill="#e74c3c" opacity="0.10"/>')
    p = u(45, g['origin_z'] - 1.8)
    e.append(txt(p[0], p[1], 'base_rear 区', 13, '#1a5276'))
    p = u(-36, g['origin_z'] - 1.8)
    e.append(txt(p[0], p[1], 'base_front 区', 13, '#922b21'))
    p = u(-40, 122)
    e.append(txt(p[0], p[1], f'立板 {g["tilt"]:g}°', 14, '#6a1b9a', rot=-20))
    p = u(-44, 58)
    e.append(txt(p[0], p[1], '键盘', 14, '#8a7d3a', rot=-20))

    # rear pad line = the tipping pivot
    pv = u(g['pad_r'], 0)
    e.append(f'<polygon points="{pv[0] - 9},{pv[1]} {pv[0] + 9},{pv[1]} '
             f'{pv[0]},{pv[1] - 12}" fill="#c0392b"/>')
    e.append(ln(pv[0] + 3, pv[1] - 2, pv[0] + 30, pv[1] + 16, '#c0392b', 0.9))
    p = u(90, -9)
    e.append(txt(p[0], p[1], '后倾支点 = 后垫线（由 base_rear 定）', 14,
                 '#c0392b'))

    # CoM + weight
    c = u(g['com_y'], g['com_z'])
    e.append(f'<circle cx="{c[0]}" cy="{c[1]}" r="5" fill="none" '
             f'stroke="#000" stroke-width="2"/>'
             f'<line x1="{c[0] - 8}" y1="{c[1]}" x2="{c[0] + 8}" y2="{c[1]}" '
             f'stroke="#000" stroke-width="2"/>'
             f'<line x1="{c[0]}" y1="{c[1] - 8}" x2="{c[0]}" '
             f'y2="{c[1] + 8}" stroke="#000" stroke-width="2"/>')
    p = u(g['com_y'] + 3, g['com_z'] + 7)
    e.append(txt(p[0], p[1], '重心', 14, halo=True))
    w0, w1 = u(g['com_y'], g['com_z'] - 6), u(g['com_y'], 20)
    e.append(f'<line x1="{w0[0]}" y1="{w0[1]}" x2="{w1[0]}" y2="{w1[1]}" '
             f'stroke="#000" stroke-width="3.5" marker-end="url(#ah)"/>')
    p = u(g['com_y'] - 6, 34)
    e.append(txt(p[0], p[1], 'W 重力', 15, anchor='end', halo=True))

    # strike force split at the top keycap
    f0, f1 = u(32, g['f_z']), u(100, g['f_z'])
    e.append(f'<line x1="{f0[0]}" y1="{f0[1]}" x2="{f1[0]}" y2="{f1[1]}" '
             f'stroke="#d62728" stroke-width="5" marker-end="url(#ahR)"/>')
    gv0, gv1 = u(32, g['f_z'] - 2), u(32, 102)
    e.append(f'<line x1="{gv0[0]}" y1="{gv0[1]}" x2="{gv1[0]}" y2="{gv1[1]}" '
             f'stroke="#1a7f37" stroke-width="5" marker-end="url(#ahG)"/>')
    p = u(44, g['f_z'] - 18)
    e.append(txt(p[0], p[1], 'F 水平分量（倾覆）', 15, '#d62728', halo=True))
    p = u(26, 120)
    e.append(txt(p[0], p[1], '竖直分量（压紧，并入 W）', 15, '#1a7f37',
                 anchor='end', halo=True))

    # sight line + h / d dims + the tip condition
    eye = u(-44, 88)
    e.append(f'<circle cx="{eye[0]}" cy="{eye[1]}" r="5" fill="none" '
             f'stroke="#555" stroke-width="1.6"/>'
             f'<circle cx="{eye[0]}" cy="{eye[1]}" r="1.8" fill="#555"/>')
    e.append(f'<line x1="{eye[0] + 7}" y1="{eye[1]}" x2="{u(6, 88)[0]}" '
             f'y2="{u(6, 88)[1]}" stroke="#555" stroke-width="1.4" '
             f'stroke-dasharray="6 4" marker-end="url(#ah)"/>')
    p = u(-44, 96)
    e.append(txt(p[0], p[1], '视线（= 本图视角）', 13, '#555'))
    xh = u(112, 0)[0]
    e.append(dim_v(xh, u(0, g['f_z'])[1], u(0, 0)[1], f'h ≈ {g["f_z"]:g}'))
    d1, d2 = u(g['com_y'] + 3, 24), u(g['pad_r'], 24)
    e.append(ln(d1[0], d1[1], d2[0], d2[1], '#333', 1.6, marker='ah'))
    e.append(txt((d1[0] + d2[0]) / 2, d1[1] - 9, 'd = 后倾力臂', 15,
                 anchor='middle', halo=True))
    p = u(-45, 172)
    e.append(txt(p[0], p[1], '不后倾：W·d ≥ F·h', 16))
    e.append(txt(W_SIDE / 2, H_SIDE - 12,
                 '← 前方　本图视角 = 使用者视线（对照 in-use 照片）　后方 →',
                 16, anchor='middle'))
    return (f'<svg x="{W_TOP}" y="0" width="{W_SIDE}" height="{H_SIDE}">'
            + ''.join(e) + '</svg>')


def build_svg(g: dict) -> str:
    height = max(H_TOP, H_SIDE) + 30
    title_a = 'C. Dux 底座：base_front / base_rear / 进深（俯视图）'
    title_b = '侧视图 ≈ 使用者站立时的视角：base_rear 为什么决定稳定性'
    return (f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{W_TOP + W_SIDE}" height="{height}" '
            f'font-family="sans-serif">{defs()}'
            f'<rect width="100%" height="100%" fill="#ffffff"/>'
            f'{txt(20, 24, title_a, 19)}'
            f'{txt(W_TOP + 16, 24, title_b, 19)}'
            f'{top_view(g)}{side_view(g)}</svg>')


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    svg_path = OUT_DIR / f'{STEM}.svg'
    svg_path.write_text(build_svg(geometry()))
    print(f'wrote {svg_path}')
    if shutil.which('rsvg-convert') is None:
        print('note: rsvg-convert not on PATH -- PNG skipped (SVG is enough)')
        return
    png_path = OUT_DIR / f'{STEM}.png'
    subprocess.run(
        ['rsvg-convert', '-w', '2860', str(svg_path), '-o', str(png_path)],
        check=True)
    print(f'wrote {png_path}')


if __name__ == '__main__':
    main()
