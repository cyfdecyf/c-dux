#!/usr/bin/env python3
"""C. Dux — vertical (dodo-style) stand, build123d implementation.

Same design and parameters as case/scad/stand.scad (see that README for the
posture / mounting / stability rationale), exported as STL (slicing) plus
STEP, the preferred format for outsourced 3D printing (JLCPCB takes exact
B-rep instead of faceted meshes).

Things this version does better than the .scad:
  * All PCB data (M4 hole positions, board outline) is parsed directly from
    the KiCad PCB file at runtime, so a board revision can never silently
    desync the stand — the script asserts against the last hand-verified
    table and refuses to run if anything moved.
  * B-rep fillets: the base slab gets a rounded top perimeter, which CSG
    mesh tools cannot do.
  * Reinforced base-plate junction for vertical typing loads: full-width
    haunch, back-face ribs, deeper embed, front tension-corner fillet.
  * Multiple mounting sets: the 4-hole pattern is repeated at several
    in-plane rotation angles, so the keyboard (and with it the home row)
    can be re-mounted at a different tilt.

Mounting orientation (per user): the single extra pinky key on its stalk
faces DOWN, the 3-key thumb cluster faces UP.

Coordinate mapping (derived, not hard-coded — see parse_pcb); both axes
flipped vs the KiCad view per the mounting orientation above:
    u = Y_max - Y_kicad          (horizontal)
    v = X_kicad - X_min          (height, from the stalk-key edge)

Rotation sets — why 10 deg steps: one post set per angle, each set being
the 4-hole pattern rotated about its centroid. 5-deg steps are impossible
with 12 mm posts: adjacent sets' identical holes land only 4-5 mm apart and
the posts would merge into a wall. 10-deg steps work when each set also
slides sideways a little (SET_U_STEP); the solver shifts per set and raises
it per set so that
  * every post keeps >= 12 mm to every other post,
  * the rotated board's lowest corner clears the base top by 2.5 mm
    (rotation swings a board corner 8-17 mm below the nominal edge),
  * each set's bottom post line stays >= 43 mm above the desk.
The sets therefore staircase slightly upward with increasing angle.

Usage:
    uv sync                                  # once; creates .venv + uv.lock
    uv run stand.py                          # left half, full (tilt fixed 20°)
    uv run stand.py --side right
    uv run stand.py --part fit               # flat validation plate + posts
    uv run stand.py --set-angles 10,20,30    # custom rotation sets
"""

from __future__ import annotations

import argparse
import itertools
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
    export_step,
    export_stl,
    extrude,
    fillet,
    mirror,
)


REPO = Path(__file__).resolve().parents[2]
DEFAULT_PCB = REPO / 'pcb' / 'v2.0' / 'c-dux.kicad_pcb'

# Last hand-verified hole table (flipped stand frame — see coordinate note
# above). The parsed values must match within TOL or the script aborts —
# that's the desync tripwire.
HOLES_EXPECTED = {
    'H1': (103.44, 127.88),  # inner top    -> upper right (stand frame)
    'H2': (41.54, 127.88),   # inner bottom -> upper left
    'H3': (85.62, 29.29),    # pinky top    -> lower right
    'H4': (35.71, 50.02),    # pinky bottom -> lower center-left
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
base_rear = 78  # rearward depth: houses the 27 mm coin slots fully + tip-over arm
base_side = 24  # wider stance against lateral rocking while typing
base_r = 10  # base slab corner radius (plan view)
# Junction reinforcement — vertical typing hammers the base-plate junction;
# the scad version only has the small embed + 3 gussets for this.
haunch_d = 18  # full-width haunch: depth along the base top
haunch_h = 25  # haunch height up the plate back face (auto-capped if a post
#               set places a low hole over the ridge)
rib_t = 8  # rib thickness (X); rib x-positions are solved automatically to
#            clear every post of every rotation set
pad_d = 20
pad_recess = 0.8
coin_x = 24
coin_w = 27
coin_h = 3.4
coin_depth = 27  # slot depth: fully buries a 25 mm coin lying flat (2 mm play)
COIN_SLOTS = False  # cut the coin ballast slots? Adds a 27 mm bridge over the
#    slot ceiling (FDM sags a little, MJF/SLS fine). Off by default: the wide
#    deep base is usually stable enough on its own; if tip-over testing says
#    otherwise, flip to True (~170 g of coin ballast) or thicken base_t
#    (8 -> 12: +~76 g of low-mounted mass, keyboard rises 4 mm).
base_top_fillet = 2  # B-rep extra: round-over on the base top perimeter

EPS = 0.01
EMBED = 4.0  # how deep the plate sinks into the base top (deeper = stronger junction)

# --- rotation sets -------------------------------------------------------------
# One post set per angle; the keyboard is re-mounted on the chosen set to
# tilt the home row. Angles are CCW in the front view — negate the list to
# mirror the tilt direction. See the module docstring for why 5-deg steps
# are impossible.
TILT = 20.0  # stand recline from vertical, deg — fixed by design
SET_ANGLES = [10, 20, 30]  # deg, CCW in front view (negate to mirror tilt)
MIN_POST_CENTER = 13.0  # post diameter 12 + 1 mm printing clearance
SHIFT_GRID = 3.0  # lateral shift candidates per set, packed by brute force
SHIFT_SPAN = 45.0  # max |lateral shift| of a set centroid
PLATE_HALF_BOUND = 72.0  # max post |x| so the plate stays on the base
BASE_TOP_MARGIN = 2.0  # rotated board's lowest point vs base top — the ONLY
#                        height constraint: clear the base, sit as low as
#                        possible (the old 43 mm bottom-line rule applied only
#                        to a base-parallel, unrotated mounting)
RIB_TARGET = 5  # ribs wanted (auto-placed; may be fewer if posts crowd the plate)

EPS = 0.01
MIN = Align.MIN


# ---------------------------------------------------------------------------
# PCB parsing — single source of truth for hole positions and board outline
# ---------------------------------------------------------------------------


def parse_pcb(path: Path) -> tuple[float, float, list[tuple[float, float]]]:
    """Return (pcb_w, pcb_h, [(u, v) x4]) from a KiCad PCB file."""
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
            u = max(ys) - y
            v = x - min(xs)
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
        post_h: float,
        attach: str,
        set_angles: list[float],
    ):
        self.pcb_w, self.pcb_h, self.holes = pcb_w, pcb_h, holes
        self.tilt = TILT
        self.post_h = post_h
        self.attach = attach

        self.origin_z = base_t - EMBED
        self.plate_h = bottom_ext + pcb_h + top_ext
        self.bx = pcb_w + 2 * base_side
        self.y0 = -base_front
        self.y1 = base_rear
        self.ct, self.st = math.cos(math.radians(TILT)), math.sin(math.radians(TILT))

        self.sets, self.set_meta = self._solve_sets(set_angles)
        self.all_posts = [(x, y) for _, pts in self.sets for x, y in pts]
        # plate must cover every post with room for the 6 mm radius + margin
        self.plate_half = max(pcb_w / 2 + plate_margin, max(abs(x) for x, _ in self.all_posts) + 8)
        self.plate_w = 2 * self.plate_half
        if self.plate_half > self.bx / 2 - 0.5:
            print(
                f'warning: rotation sets need plate half-width {self.plate_half:.1f} '
                f'but the base is only {self.bx / 2:.1f} wide — plate overhangs the base'
            )
        self.rib_x = self._auto_ribs()
        if len(self.rib_x) < 3:
            print(f'warning: only {len(self.rib_x)} ribs fit around the post sets')
        self.haunch_h = self._cap_haunch()

    # -- rotation-set layout solver ------------------------------------------

    def _solve_sets(self, angles: list[float]) -> list[tuple[float, list[tuple[float, float]]]]:
        """Place one 4-hole set per angle: rotate about the pattern centroid,
        then pack — each set slides laterally (brute force over a shift grid)
        and rises so that (a) the rotated board's lowest corner clears the
        base top, (b) its bottom post line stays LINE_H_DESK above the desk,
        and (c) all posts keep MIN_POST_CENTER apart. Optimizes for the
        largest minimum post distance, tie-broken by a compact span."""
        cu = sum(u for u, _ in self.holes) / len(self.holes)
        cv = sum(v for _, v in self.holes) / len(self.holes)
        rel = [(u - cu, v - cv) for u, v in self.holes]
        corners = [
            (-self.pcb_w / 2, -cv),
            (self.pcb_w / 2, -cv),
            (self.pcb_w / 2, self.pcb_h - cv),
            (-self.pcb_w / 2, self.pcb_h - cv),
        ]
        # v' the lowest board corner may reach: the plate back face at that
        # corner must stay BASE_TOP_MARGIN above the base top
        v_floor = (base_t + BASE_TOP_MARGIN - self.origin_z - 11 * self.st) / self.ct - bottom_ext

        lifted = []
        for ang in angles:
            r = math.radians(ang)
            pts = [
                (cu + dx * math.cos(r) - dy * math.sin(r),
                 cv + dx * math.sin(r) + dy * math.cos(r))
                for dx, dy in rel
            ]
            vmin = min(dx * math.sin(r) + dy * math.cos(r) for dx, dy in corners)
            lift = max(0.0, v_floor - (cv + vmin))
            lifted.append((ang, lift, [(u, v + lift) for u, v in pts]))

        cands = [
            s * SHIFT_GRID
            for s in range(int(-SHIFT_SPAN / SHIFT_GRID), int(SHIFT_SPAN / SHIFT_GRID) + 1)
        ]
        best, best_key = None, None
        for combo in itertools.product(cands, repeat=len(angles)):
            posts = [
                (u + s - self.pcb_w / 2, v + bottom_ext)
                for (_, _, pts), s in zip(lifted, combo)
                for u, v in pts
            ]
            if max(abs(x) for x, _ in posts) > PLATE_HALF_BOUND - 8:
                continue
            dmin = min(
                math.dist(posts[i], posts[j])
                for i in range(len(posts))
                for j in range(i + 1, len(posts))
            )
            if dmin < MIN_POST_CENTER:
                continue
            key = (min(dmin, 20.0), -(max(combo) - min(combo)))
            if best_key is None or key > best_key:
                best, best_key = combo, key
        if best is None:
            sys.exit(
                'error: no post arrangement fits these rotation angles — '
                'try fewer sets or smaller steps (--set-angles)'
            )
        out = []
        meta = []
        for (ang, lift, pts), s in zip(lifted, best):
            r = math.radians(ang)
            vmin = min(cv + dx * math.sin(r) + dy * math.cos(r) for dx, dy in corners)
            corner_z = self.origin_z + (bottom_ext + vmin + lift) * self.ct + 11 * self.st
            out.append((ang, [(u + s - self.pcb_w / 2, v + bottom_ext) for u, v in pts]))
            meta.append((ang, s, lift, round(corner_z, 2)))
        return out, meta

    # -- rib / haunch auto-placement ------------------------------------------

    def _auto_ribs(self) -> list[float]:
        """Rib x positions: farthest-point sampling over the x candidates that
        keep >= 12 mm sideways to every post passing through the rib band."""
        band_lo, band_hi = 3 - 6, 63.4 + 6  # rib band incl. post radius
        zone = [(x, y) for x, y in self.all_posts if band_lo <= y <= band_hi]
        lo, hi = -(self.plate_half - 12), self.plate_half - 12
        cand = []
        for i in range(int((hi - lo) / 2) + 1):
            x = lo + 2 * i
            if all(abs(x - px) >= 12 for px, _ in zone):
                cand.append(x)
        if not cand:
            return []
        chosen = [max(cand)]  # start at the left edge of the free strip
        while len(chosen) < RIB_TARGET:
            nxt = max(cand, key=lambda x: min(abs(x - c) for c in chosen))
            if min(abs(nxt - c) for c in chosen) < 15:
                break
            chosen.append(nxt)
        return sorted(chosen)

    def _cap_haunch(self) -> float:
        """Cap the haunch height so its ridge never runs behind a post's
        screw clearance hole on the plate back face."""
        min_y = min(y for _, y in self.all_posts)
        y_top = min_y - 3
        h = (self.origin_z + y_top * self.ct - base_t + 1) / self.ct
        return max(10.0, min(haunch_h, h))

    # -- keyboard-plate frame helpers -----------------------------------------

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
        for hole in self.all_posts:
            x, y = hole
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
        for hole in self.all_posts:
            x, y = hole
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
        if COIN_SLOTS:
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

        # Full-width (= plate width) haunch: fills the junction inner corner
        # so typing loads bend a shallow ramp instead of prying open a
        # knife-edge corner. Kept flush with the plate sides — wider than
        # that, the haunch apex would stand exposed as a fragile fin.
        base += Pos(-self.plate_w / 2, -2, base_t - 1) * extrude(
            Plane.YZ * wedge(haunch_d, self.haunch_h), self.plate_w
        )
        # Ribs continue up the back face into the hammering zone.
        for rx in self.rib_x:
            base += Pos(rx - rib_t / 2, -2, base_t - 1) * extrude(
                Plane.YZ * wedge(rib_t + 14, 60), rib_t
            )
        return base

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
            + Pos(0, 0, self.origin_z)
            * Rot(X=90 - self.tilt)
            * self.plate_and_posts()
        )
        body = self.fillet_front_junction(body)
        if side == 'right':
            body = mirror(body, about=Plane.YZ)
        return body


front_fillet_r = 2.5  # radius on the tension-side (front) junction corner


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
    ap.add_argument(
        '--set-angles',
        type=str,
        default=','.join(str(a) for a in SET_ANGLES),
        help='comma-separated rotation angles (deg) of the mounting sets',
    )
    args = ap.parse_args()

    set_angles = [float(a) for a in args.set_angles.split(',')]
    if len(set_angles) < 1:
        sys.exit('error: --set-angles needs at least one angle')

    pcb_w, pcb_h, holes = parse_pcb(args.pcb)
    stand = Stand(pcb_w, pcb_h, holes, args.post_h, args.attach, set_angles)

    for (ang, pts), (_, _, _, corner_z) in zip(stand.sets, stand.set_meta):
        print(f'set {ang:5.1f}°: posts x=[{min(x for x, _ in pts):7.2f}, '
              f'{max(x for x, _ in pts):7.2f}]  lowest PCB corner z={corner_z:6.2f} mm')
    print(f'ribs at x = {[round(x, 1) for x in stand.rib_x]}  | haunch_h = {stand.haunch_h:.1f} mm')

    part = stand.build(args.part, args.side)
    stem = f'stand_{args.side}_{args.part}'
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
