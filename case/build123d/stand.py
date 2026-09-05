#!/usr/bin/env python3
"""C. Dux — vertical (dodo-style) stand, build123d implementation.

Same design and parameters as case/scad/stand.scad (see that README for the
posture / mounting / stability rationale), exported as STL (slicing) plus
STEP, the preferred format for outsourced 3D printing (JLCPCB takes exact
B-rep instead of faceted meshes).

Three things this version does better than the .scad:
  * All PCB data (M4 hole positions, board outline) is parsed directly from
    the KiCad PCB file at runtime, so a board revision can never silently
    desync the stand — the script asserts against the last hand-verified
    table and refuses to run if anything moved.
  * B-rep fillets: the base slab gets a rounded top perimeter, which CSG
    mesh tools cannot do.
  * Reinforced base-plate junction for vertical typing loads: full-width
    haunch, back-face ribs, deeper embed, front tension-corner fillet.

Coordinate mapping (derived, not hard-coded — see parse_pcb):
    u = Y_kicad - Y_min          (horizontal, from the "top row" long edge)
    v = X_max - X_kicad          (height, from the controller inner edge)

Usage:
    uv sync                                  # once; creates .venv + uv.lock
    uv run stand.py                          # left half, tilt=20, full
    uv run stand.py --side right --tilt 0
    uv run stand.py --part fit               # flat validation plate + posts
"""

from __future__ import annotations

import argparse
import math
import re
import sys

from pathlib import Path

from build123d import (
    Align,
    Axis,
    Box,
    Cone,
    Cylinder,
    Plane,
    Polygon,
    Pos,
    RectangleRounded,
    RegularPolygon,
    Rot,
    Text,
    export_step,
    export_stl,
    extrude,
    fillet,
    mirror,
)


REPO = Path(__file__).resolve().parents[2]
DEFAULT_PCB = REPO / 'pcb' / 'v2.0' / 'c-dux.kicad_pcb'

# Last hand-verified hole table (case/scad/README.md). The parsed values must
# match these within TOL or the script aborts — that's the desync tripwire.
HOLES_EXPECTED = {
    'H1': (9.06, 26.12),  # inner top    -> stand bottom left
    'H2': (70.96, 26.12),  # inner bottom -> stand bottom right
    'H3': (26.88, 124.71),  # pinky top    -> stand upper left
    'H4': (76.79, 103.98),  # pinky bottom -> stand upper right
}
PCB_W_EXPECTED = 112.5
PCB_H_EXPECTED = 154.0
HOLE_TOL = 0.05
SIZE_TOL = 0.5

# --- geometry parameters, same names/defaults as case/scad/stand.scad ---------
plate_t = 6
plate_margin = 10
bottom_ext = 6.5  # plate below the PCB bottom edge; with base_t=8 the tilt=0
#                  posture clears the base top by 2.5 mm (the tightest case)
top_ext = 12
plate_r = 8
post_d = 12
nut_af = 7.4
nut_h = 3.4
screw_d = 4.5
tap_d = 3.4
base_t = 8  # lower base top = lower keyboard; 8 keeps 2.3 mm walls around the coin slots
base_front = 40  # deeper towards the user: support polygon under the hands
base_rear = 65
base_side = 24  # wider stance against lateral rocking while typing
base_r = 10
# Junction reinforcement — vertical typing hammers every keypress through
# the backplate into the base-plate junction; the scad version only has the
# small embed + 3 gussets for this.
haunch_d = 18  # full-width haunch: depth along the base top
haunch_h = 25  # haunch height up the plate back face
rib_x = [-52, -26, 0, 26, 52]
rib_t = 8  # rib thickness (X)
rib_d = 22  # rib depth along the base top
rib_h = 60  # rib height up the plate back face
front_fillet_r = 2.5  # radius on the tension-side (front) junction corner
pad_d = 20
pad_recess = 0.8
coin_x = 24
coin_w = 27
coin_h = 3.4
coin_depth = 14
label_text = 'C.DUX'
base_top_fillet = 2  # B-rep extra: round-over on the base top perimeter

EPS = 0.01
EMBED = 4.0  # how deep the plate sinks into the base top (deeper = stronger junction)

MIN = Align.MIN


# ---------------------------------------------------------------------------
# PCB parsing — single source of truth for hole positions and board outline
# ---------------------------------------------------------------------------


def parse_pcb(path: Path) -> tuple[float, float, list[tuple[float, float]]]:
    """Return (pcb_w, pcb_h, [(u, v) per H1..H4]) from a KiCad PCB file."""
    text = path.read_text()

    # Board outline: ergogen exports Edge.Cuts as gr_line/gr_arc segments
    # (arcs tessellated into short lines, so endpoints suffice for the bbox).
    xs: list[float] = []
    ys: list[float] = []
    for line in text.splitlines():
        if 'Edge.Cuts' not in line:  # KiCad writes the layer name unquoted
            continue
        if not line.strip().startswith('(gr_'):
            continue
        # start/end/center only: gr_arc "mid" points are corner-rounding
        # bulges outside the straight-edge hull, and the stand frame (like
        # the verified hole table) is anchored to the straight-edge corners.
        for a, b in re.findall(r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line):
            xs.append(float(a))
            ys.append(float(b))
    if not xs:
        sys.exit(f'error: no Edge.Cuts geometry found in {path}')
    pcb_h = max(xs) - min(xs)  # board height (along columns) runs along X in KiCad
    pcb_w = max(ys) - min(ys)

    holes: dict[str, tuple[float, float]] = {}
    chunks = text.split('(footprint "ceoloide:mounting_hole_plated"')[1:]
    for chunk in chunks:
        head = chunk[:1500]
        at = re.search(r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)', head)
        ref = re.search(r'\(property "Reference"\s+"([^"]+)"', head)
        if at and ref:
            x, y = float(at.group(1)), float(at.group(2))
            u = y - min(ys)
            v = max(xs) - x
            holes[ref.group(1)] = (u, v)

    missing = set(HOLES_EXPECTED) - set(holes)
    if missing:
        sys.exit(f'error: mounting holes {sorted(missing)} not found in {path}')

    for ref, expected in HOLES_EXPECTED.items():
        got = holes[ref]
        if any(abs(g - e) > HOLE_TOL for g, e in zip(got, expected)):
            sys.exit(
                f'error: hole {ref} moved: parsed {got} vs expected {expected}. '
                'The PCB no longer matches the verified table — update '
                'HOLES_EXPECTED after checking the new positions.'
            )
    if abs(pcb_w - PCB_W_EXPECTED) > SIZE_TOL or abs(pcb_h - PCB_H_EXPECTED) > SIZE_TOL:
        sys.exit(
            f'error: board outline changed: parsed {pcb_w:.2f} x {pcb_h:.2f} '
            f'vs expected {PCB_W_EXPECTED} x {PCB_H_EXPECTED}'
        )
    ordered = [holes[ref] for ref in sorted(HOLES_EXPECTED)]
    return pcb_w, pcb_h, ordered


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


class Stand:
    def __init__(
        self,
        pcb_w: float,
        pcb_h: float,
        holes: list[tuple[float, float]],
        tilt: float,
        post_h: float,
        attach: str,
    ):
        self.pcb_w, self.pcb_h, self.holes = pcb_w, pcb_h, holes
        self.tilt = tilt
        self.post_h = post_h
        self.attach = attach

        self.embed_origin_z = base_t - EMBED
        self.plate_w = pcb_w + 2 * plate_margin
        self.plate_h = bottom_ext + pcb_h + top_ext
        self.bx = pcb_w + 2 * base_side
        self.y0 = -base_front
        self.y1 = base_rear

    def hole_xy(self, hole: tuple[float, float]) -> tuple[float, float]:
        u, v = hole
        return u - self.pcb_w / 2, v + bottom_ext

    def pad_positions(self) -> list[tuple[float, float]]:
        px = self.bx / 2 - base_r - 4
        fy = self.y0 + base_r + 4
        ry = self.y1 - base_r - 4
        return [(-px, fy), (px, fy), (0, fy), (-px, ry), (px, ry), (0, ry)]

    # -- backplate + posts, plate-local frame (x right, y up, z out front) ----

    def plate_and_posts(self):
        solid = Pos(0, self.plate_h / 2, 0) * extrude(
            RectangleRounded(self.plate_w, self.plate_h, plate_r), plate_t
        )
        for hole in self.holes:
            x, y = self.hole_xy(hole)
            solid += Pos(x, y, plate_t) * (
                Cylinder(post_d / 2, self.post_h, align=(Align.CENTER, Align.CENTER, MIN))
                + Cone(
                    post_d / 2 + 2.5,
                    post_d / 2,
                    1.5,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
            )
        top = plate_t + self.post_h
        for hole in self.holes:
            x, y = self.hole_xy(hole)
            if self.attach == 'nut':
                # Hex pocket open at the post top: drop the M4 nut in, drive
                # the screw through the PCB into it.
                hex_r = (nut_af / 2) / math.cos(math.radians(30))
                solid -= Pos(x, y, top - nut_h) * extrude(
                    RegularPolygon(hex_r, 6, major_radius=True), nut_h + EPS
                )
                solid -= Pos(x, y, 0) * Cylinder(
                    screw_d / 2,
                    top - nut_h + EPS,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
            else:
                solid -= Pos(x, y, 2) * Cylinder(
                    tap_d / 2, top - 2, align=(Align.CENTER, Align.CENTER, MIN)
                )
        return solid

    # -- base, world frame (desk = XY plane, user at -Y) ----------------------

    def base_solid(self):
        base = Pos(0, (self.y0 + self.y1) / 2, base_t / 2) * Box(
            self.bx, self.y1 - self.y0, base_t
        )
        base = fillet(base.edges().filter_by(Axis.Z), base_r)
        base = fillet(base.edges().group_by(Axis.Z)[-1], base_top_fillet)
        for px, py in self.pad_positions():
            base -= Pos(px, py, -EPS) * Cylinder(
                pad_d / 2, pad_recess + EPS, align=(Align.CENTER, Align.CENTER, MIN)
            )
        for sx in (-1, 1):
            base -= Pos(
                sx * coin_x - coin_w / 2, self.y1 - coin_depth, base_t / 2 - coin_h / 2
            ) * Box(
                coin_w,
                coin_depth + 1,
                coin_h,
                align=(MIN, MIN, MIN),
            )
        rad = math.radians(self.tilt)

        def wedge(depth: float, height: float):
            # Profile in the YZ plane: base along the base top, apex up the
            # tilted backplate's back face; anchor edges sit inside the
            # solids so the union fuses cleanly, and every face is
            # support-free as printed.
            return Polygon(
                (0, 0),
                (depth, 0),
                (height * math.sin(rad), height * math.cos(rad)),
                align=None,
            )

        # Full-width(= plate width)haunch: fills the junction inner corner
        # so typing loads bend a shallow ramp instead of prying open a
        # knife-edge corner. Kept flush with the plate sides — wider than
        # that, the haunch apex would stand exposed as a fragile fin.
        base += Pos(-self.plate_w / 2, -2, base_t - 1) * extrude(
            Plane.YZ * wedge(haunch_d, haunch_h), self.plate_w
        )
        # Ribs continue up the back face into the hammering zone (thumb /
        # bottom rows sit just above the junction).
        for rx in rib_x:
            base += Pos(rx - rib_t / 2, -2, base_t - 1) * extrude(
                Plane.YZ * wedge(rib_d, rib_h), rib_t
            )
        return base

    def emboss_label(self, part):
        if not label_text:
            return part
        try:
            # Front face of the base, normal towards the user (-Y). Built as
            # one Plane — mixing Pos with a Plane composes the offset in the
            # plane's local frame, which silently misplaces the text.
            plane = Plane((0, self.y0, 2), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
            sketch = plane * Text(label_text, font_size=6, align=(Align.CENTER, MIN))
        except Exception as exc:  # no usable system font — skip, not fatal
            print(f'warning: label skipped ({exc})')
            return part
        return part + extrude(sketch, 0.9)

    def fillet_front_junction(self, body):
        """Round the concave edge where the plate front face meets the base
        top. Typing pries the plate forward, so this tension-side corner
        should not stay a knife edge."""
        edges = [
            e
            for e in body.edges().filter_by(Axis.X)
            if abs(e.center().Z - base_t) < 0.2
            and -7 < e.center().Y < -1
            and e.length > 50  # skip slivers beside the plate's rounded corners
        ]
        if not edges:
            print('warning: front junction edge not found — fillet skipped')
            return body
        try:
            return fillet(edges, front_fillet_r)
        except Exception as exc:
            print(f'warning: front junction fillet failed ({exc}) — skipped')
            return body

    # -- top level ------------------------------------------------------------

    def build(self, part: str, side: str):
        if part == 'fit':
            body = self.plate_and_posts()
            return mirror(body, about=Plane.YZ) if side == 'right' else body
        body = (
            self.base_solid()
            + Pos(0, 0, self.embed_origin_z)
            * Rot(X=90 - self.tilt)
            * self.plate_and_posts()
        )
        body = self.fillet_front_junction(body)
        if side == 'right':
            body = mirror(body, about=Plane.YZ)
        return self.emboss_label(body)


def summary(name: str, part) -> None:
    bb = part.bounding_box()
    sx, sy, sz = (bb.size.X, bb.size.Y, bb.size.Z)
    vol_cm3 = part.volume / 1000
    print(
        f'{name}: bbox {sx:.1f} x {sy:.1f} x {sz:.1f} mm | '
        f'solid {vol_cm3:.0f} cm³ | PETG ≈ {vol_cm3 * 1.27:.0f} g, '
        f'PA12 ≈ {vol_cm3 * 1.01:.0f} g (solid, no infill allowance)'
    )


def push_viewer(part, name: str) -> None:
    """Send the model to the OCP CAD Viewer (VS Code). Best effort, never
    fatal — exporting must not depend on the viewer running."""
    try:
        from ocp_vscode import Camera, show
    except ImportError:
        print('note: viewer push skipped — install with: uv add --dev ocp-vscode')
        return
    try:
        show(part, names=[name], reset_camera=Camera.CENTER)
        print(f'pushed to OCP CAD Viewer: {name}')
    except Exception as exc:
        print(f'note: viewer push skipped ({type(exc).__name__}: {exc})')


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--side', choices=['left', 'right'], default='left')
    ap.add_argument(
        '--part',
        choices=['full', 'fit'],
        default='full',
        help="'fit' = flat validation plate + posts only",
    )
    ap.add_argument(
        '--tilt',
        type=float,
        default=20,
        help='recline from vertical, degrees (0 = upright)',
    )
    ap.add_argument(
        '--post-h', type=float, default=5, help='post height = PCB-to-plate gap'
    )
    ap.add_argument(
        '--attach',
        choices=['nut', 'tap'],
        default='nut',
        help='M4 nut pocket (nut) or self-tapping pilot (tap)',
    )
    ap.add_argument(
        '--pcb',
        type=Path,
        default=DEFAULT_PCB,
        help='KiCad PCB file to parse hole positions from',
    )
    ap.add_argument('--out', type=Path, default=Path(__file__).parent / 'out')
    ap.add_argument(
        '--show',
        action='store_true',
        help='push to the OCP CAD Viewer instead of exporting',
    )
    args = ap.parse_args()

    pcb_w, pcb_h, holes = parse_pcb(args.pcb)
    stand = Stand(pcb_w, pcb_h, holes, args.tilt, args.post_h, args.attach)

    part = stand.build(args.part, args.side)
    stem = (
        f'stand_{args.side}_tilt{int(round(args.tilt))}_{args.part}'
        if args.part == 'full'
        else f'stand_{args.side}_fit'
    )
    summary(stem, part)
    if args.show:
        push_viewer(part, stem)
        return
    args.out.mkdir(parents=True, exist_ok=True)
    stl, step = args.out / f'{stem}.stl', args.out / f'{stem}.step'
    export_stl(part, stl)
    export_step(part, step)
    print(f'exported: {stl}\n          {step}')
    push_viewer(part, stem)


if __name__ == '__main__':
    main()
