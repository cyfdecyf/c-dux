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
  * Demountable two-part design: the base slab is printed ONCE (it is
    symmetric and shared by both sides and every tilt angle); the upright
    (backplate + posts + boot foot) is printed per tilt angle. The joint is
    two shear keys plus 2x M4x16 socket screws into nut pockets — swapping
    angles means reprinting only the upright.
  * Junction reinforcement survives the split: the full-width haunch and
    the back-face ribs moved onto the upright's boot.
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
    uv run stand.py --part base              # shared base slab (print once)
    uv run stand.py --part upright           # left upright, 20 deg recline
    uv run stand.py --side right --part upright
    uv run stand.py --part upright --tilt 10,15,20,25,30   # a batch of angles
    uv run stand.py --part full              # fused assembly preview
    uv run stand.py --part joint             # small fit-test coupons
    uv run stand.py --part fit               # legacy flat validation plate
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
    chamfer,
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
base_t = 8  # the seat plane: base top = upright's boot bottom
base_front = 40  # deeper towards the user: support polygon under the hands
base_rear = 100  # rearward depth on the tilt side: longer tip-over lever arm
base_side = 24  # wider stance against lateral rocking while typing
base_r = 10  # base slab corner radius (plan view)
# Junction reinforcement — vertical typing hammers the base-upright junction;
# the junction itself is now a demountable joint (constants below).
haunch_d = 18  # full-width haunch: depth along the seat plane
haunch_h = 10  # haunch height above the seat plane: a shallow ramp that
#               tops out just above the boot, so the 2 joint screws keep
#               their counterbores on the open boot ledge (the taller
#               reinforcement is the ribs' job)
rib_t = 8  # rib thickness (X); rib x-positions are solved automatically to
#            clear every post of every rotation set AND the joint hardware
rib_h = 60  # rib height up the back face
pad_d = 20
pad_recess = 0.8
base_top_fillet = 2  # B-rep extra: round-over on the base top perimeter

# --- base ↔ upright joint -------------------------------------------------------
# The stand ships as two printed parts: the base slab (printed once — shared
# by every tilt angle and both sides) and a per-angle upright (backplate +
# posts + boot foot). Every joint feature is laid out in WORLD coordinates on
# the horizontal seat plane z = base_t, so one base accepts uprights of any
# tilt. Assembly per side: drop the upright onto the two keys, drive 2x
# M4x16 socket screws from the boot top through into nut pockets in the base
# underside. The seat plane carries compression, the keys carry shear and
# location, the screws carry lift and clamp.
TILT = 20.0  # stand recline from vertical, deg — per-upright (--tilt);
#             first batch prints the 20° pair only
joint_clear = 0.4  # mating clearance per face (README tolerance baseline)
key_l = 30  # shear key footprint: key_l along X, key_w along Y, key_h tall
key_w = 8
key_h = 3.2  # shallower than its groove (key_h + joint_clear): the boot
#             seats on the base top plane, never bottoming out on the keys
key_x = 12  # key centerline |x|; keys span y ∈ (key_y0, key_y1)
key_y0, key_y1 = 12, 20
key_chamfer = 0.5  # 45° lead-in on the key top edges
boot_h = 6  # boot foot height; its top ledge carries the counterbored heads
boot_front = -2  # boot front edge — behind the plate front face at the seat
boot_rear = 20  # boot rear edge (= rib wedge footprint depth)
joint_screw_d = 4.5  # M4 clearance through boot + base
joint_screw_x = 58  # on the boot ledge for every tilt in 10..40 deg; must
#                    clear the keys and the ribs (see the rib keep-outs)
joint_screw_y = 14.5
cbore_d = 8.5  # M4 socket head, sunk flush with the boot top
cbore_h = 4.0
joint_screw = 'M4x16'  # informational — printed in the BOM note, not geometry
nut_mouth_w = 6.9  # nut-pocket mouth pinched below the nut's 7.0 across
#                   flats: push the nut in past the flexing lip; it stays
nut_mouth_h = 1.2

# --- rotation sets -------------------------------------------------------------
# One post set per angle; the keyboard is re-mounted on the chosen set to
# tilt the home row. Angles are CCW in the front view — negate the list to
# mirror the tilt direction. See the module docstring for why 5-deg steps
# are impossible.
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
# Joint geometry — world frame, defined once so the base accepts uprights of
# any tilt (and so the --part joint coupons reuse the exact production
# features instead of a lookalike)
# ---------------------------------------------------------------------------


def joint_keys():
    """Two shear keys standing on the seat plane. They locate the upright in
    X/Y/yaw and carry the horizontal typing loads; the seat plane carries
    compression, the screws carry lift and clamp."""
    keys = None
    for sx in (-1, 1):
        key = Pos(sx * key_x - key_l / 2, key_y0, base_t) * Box(
            key_l, key_w, key_h, align=(MIN, MIN, MIN)
        )
        keys = key if keys is None else keys + key
    # 45° lead-in all around the top so the boot grooves drop on cleanly
    return chamfer(keys.edges().group_by(Axis.Z)[-1], key_chamfer)


def joint_screw_bores():
    """Vertical M4 clearance holes through the base (pair with
    joint_nut_pockets on the underside)."""
    bores = None
    for sx in (-1, 1):
        bore = Pos(sx * joint_screw_x, joint_screw_y, -EPS) * Cylinder(
            joint_screw_d / 2,
            base_t + 2 * EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        bores = bore if bores is None else bores + bore
    return bores


def joint_nut_pockets():
    """Hex pockets open at the base underside, the mouth pinched just below
    the nut's across-flats: push the nut in past the flexing lip and it
    stays put for every future assembly."""
    hex_r = (nut_af / 2) / math.cos(math.radians(30))
    pockets = None
    for sx in (-1, 1):
        pocket = Pos(sx * joint_screw_x, joint_screw_y, -EPS) * (
            extrude(RegularPolygon(hex_r, 6, major_radius=True), nut_h + EPS)
            + Box(
                nut_mouth_w,
                nut_mouth_w,
                nut_mouth_h + EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def joint_grooves():
    """Matching pockets in the boot underside, joint_clear deeper than the
    keys are tall so the boot seats on the base top, not on the keys."""
    grooves = None
    for sx in (-1, 1):
        groove = Pos(
            sx * key_x - key_l / 2 - joint_clear,
            key_y0 - joint_clear,
            base_t,
        ) * Box(
            key_l + 2 * joint_clear,
            key_w + 2 * joint_clear,
            key_h + joint_clear,
            align=(MIN, MIN, MIN),
        )
        grooves = groove if grooves is None else grooves + groove
    return grooves


def joint_screw_heads():
    """Boot-side screw features: clearance through the boot plus a
    counterbore so an M4 socket head sits flush with the boot top."""
    holes = None
    for sx in (-1, 1):
        hole = Pos(sx * joint_screw_x, joint_screw_y, base_t) * (
            Cylinder(
                joint_screw_d / 2,
                boot_h + EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
            + Pos(0, 0, boot_h - cbore_h)
            * Cylinder(
                cbore_d / 2,
                cbore_h + EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
        )
        holes = hole if holes is None else holes + hole
    return holes


def joint_coupons():
    """Pair of mating fit-test coupons — a base fragment (key + screw bore +
    nut pocket) and a boot fragment (groove + counterbore) — printed flat
    before committing to full parts: the 0.4 mm clearance and the nut lip
    are the two things worth checking on your machine first. Both lie on
    the bed, 40 mm apart."""
    frag_y0, frag_y1 = key_y0 - 18, key_y1 + 6
    base_c = Pos(0, (frag_y0 + frag_y1) / 2, base_t / 2) * Box(
        2 * (joint_screw_x + 12), frag_y1 - frag_y0, base_t
    )
    base_c += joint_keys()
    base_c -= joint_screw_bores() + joint_nut_pockets()
    boot_c = Pos(0, (frag_y0 + frag_y1) / 2, base_t + boot_h / 2) * Box(
        2 * (joint_screw_x + 12), frag_y1 - frag_y0, boot_h
    )
    boot_c -= joint_grooves() + joint_screw_heads()
    # drop the boot fragment onto the bed next to the base fragment
    return base_c + Pos(0, frag_y1 - frag_y0 + 40, -base_t) * boot_c


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
        tilt: float = TILT,
    ):
        self.pcb_w, self.pcb_h, self.holes = pcb_w, pcb_h, holes
        self.tilt = tilt
        self.post_h = post_h
        self.attach = attach

        self.origin_z = base_t  # the plate rests on the seat plane; the boot
        #                         foot + keys + joint screws carry the junction
        self.plate_h = bottom_ext + pcb_h + top_ext
        self.bx = pcb_w + 2 * base_side
        self.y0 = -base_front
        self.y1 = base_rear
        self.ct, self.st = math.cos(math.radians(tilt)), math.sin(math.radians(tilt))

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
        if self.plate_half < joint_screw_x + 5:
            sys.exit(
                f'error: joint screws at |x|={joint_screw_x:g} need plate half-width '
                f'>= {joint_screw_x + 5:g}, got {self.plate_half:.1f} — reduce the '
                '--set-angles spread'
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
        keep >= 12 mm sideways to every post passing through the rib band and
        clear of the joint hardware crossing the boot bottom (keys at
        |x| <= key_x + key_l/2, screw counterbores around joint_screw_x)."""
        band_lo, band_hi = 3 - 6, rib_h + 6  # rib band incl. post radius
        zone = [(x, y) for x, y in self.all_posts if band_lo <= y <= band_hi]
        lo, hi = -(self.plate_half - 12), self.plate_half - 12
        cand = []
        for i in range(int((hi - lo) / 2) + 1):
            x = lo + 2 * i
            if abs(x) < key_x + key_l / 2 + rib_t / 2 + 1:
                continue
            if abs(abs(x) - joint_screw_x) < cbore_d / 2 + rib_t / 2 + 1:
                continue
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
        base += joint_keys()
        base -= joint_screw_bores() + joint_nut_pockets()
        return base

    # -- upright: backplate + posts + boot foot, one part per tilt angle ------

    def upright_solid(self):
        """The demountable half: everything above the seat plane, printed
        boot-down on the bed, support-free like the old one-piece print."""
        rad = math.radians(self.tilt)
        body = (
            Pos(0, 0, self.origin_z)
            * Rot(X=90 - self.tilt)
            * self.plate_and_posts()
        )
        boot = Pos(0, (boot_front + boot_rear) / 2, base_t + boot_h / 2) * Box(
            self.plate_w, boot_rear - boot_front, boot_h
        )
        # Full-width haunch: a shallow ramp up the back face that tops out
        # just above the boot (haunch_h), filling the junction inner corner.
        boot += Pos(-self.plate_w / 2, boot_front, base_t) * extrude(
            Plane.YZ * self._wedge(haunch_d, self.haunch_h, rad), self.plate_w
        )
        # Ribs continue up the back face into the hammering zone.
        for rx in self.rib_x:
            boot += Pos(rx - rib_t / 2, boot_front, base_t) * extrude(
                Plane.YZ * self._wedge(rib_t + 14, rib_h, rad), rib_t
            )
        body += boot
        body -= joint_grooves()
        body -= joint_screw_heads()
        # Trims the bottom post flares, which dip below the seat plane at
        # low tilt angles (nothing else lives down there).
        body -= Pos(0, 0, base_t - 25) * Box(400, 400, 50)
        return body

    def _wedge(self, depth: float, height: float, rad: float):
        # Profile in the YZ plane: base along the seat plane, apex up the
        # tilted backplate's back face; anchor edges sit inside the
        # solids so the union fuses cleanly, and every face is
        # support-free as printed.
        return Polygon(
            (0, 0),
            (depth, 0),
            (height * math.sin(rad), height * math.cos(rad)),
            align=None,
        )

    # -- top level ------------------------------------------------------------

    def build(self, part: str, side: str):
        if part == 'base':
            return self.base_solid()
        if part == 'fit':
            body = self.plate_and_posts()
            return mirror(body, about=Plane.YZ) if side == 'right' else body
        if part == 'upright':
            body = self.upright_solid()
            return mirror(body, about=Plane.YZ) if side == 'right' else body
        if part == 'joint':
            return joint_coupons()
        # 'full': fused assembly preview of the two parts
        body = self.base_solid() + self.upright_solid()
        return mirror(body, about=Plane.YZ) if side == 'right' else body


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
        choices=['base', 'upright', 'full', 'fit', 'joint'],
        default='full',
        help="'base' = shared slab (print once) | 'upright' = per-tilt part | "
        "'full' = fused preview | 'fit' = flat validation plate | "
        "'joint' = fit-test coupons",
    )
    ap.add_argument(
        '--tilt',
        type=str,
        default=f'{TILT:g}',
        help='comma-separated stand recline angles (deg) for upright/full',
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
    tilts = [float(a) for a in args.tilt.split(',')]
    if len(tilts) < 1:
        sys.exit('error: --tilt needs at least one angle')

    pcb_w, pcb_h, holes = parse_pcb(args.pcb)
    args.out.mkdir(parents=True, exist_ok=True)

    def emit(part, stem: str) -> None:
        summary(stem, part)
        if args.show:
            push_viewer(part, stem)
            return
        stl, step = args.out / f'{stem}.stl', args.out / f'{stem}.step'
        export_stl(part, stl)
        export_step(part, step)
        print(f'exported: {stl}\n          {step}')
        push_viewer(part, stem)

    if args.part == 'joint':
        emit(joint_coupons(), 'joint_coupons')
        return

    # The base is tilt- and side-independent: one Stand, one export.
    if args.part == 'base':
        stands = [Stand(pcb_w, pcb_h, holes, args.post_h, args.attach, set_angles)]
        stems = ['stand_base']
    else:
        # 'fit' geometry does not vary with the tilt; only build it once.
        if args.part == 'fit':
            tilts = tilts[:1]
        stands, stems = [], []
        for tilt in tilts:
            stands.append(
                Stand(pcb_w, pcb_h, holes, args.post_h, args.attach, set_angles, tilt)
            )
            tag = f'_t{tilt:g}'
            if args.part == 'upright':
                stems.append(f'stand_{args.side}_upright{tag}')
            elif args.part == 'full':
                stems.append(f'stand_{args.side}_full{tag}')
            else:
                stems.append(f'stand_{args.side}_fit')

    for stand, stem in zip(stands, stems, strict=True):
        for (ang, pts), (_, _, _, corner_z) in zip(stand.sets, stand.set_meta):
            print(
                f'set {ang:5.1f}°: posts x=[{min(x for x, _ in pts):7.2f}, '
                f'{max(x for x, _ in pts):7.2f}]  lowest PCB corner z={corner_z:6.2f} mm'
            )
        print(
            f'ribs at x = {[round(x, 1) for x in stand.rib_x]}'
            f'  | haunch_h = {stand.haunch_h:.1f} mm'
            f'  | joint: keys x=±{key_x:g}, screws x=±{joint_screw_x:g} ({joint_screw})'
        )
        emit(stand.build(args.part, args.side), stem)


if __name__ == '__main__':
    main()
