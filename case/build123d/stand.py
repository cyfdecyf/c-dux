#!/usr/bin/env python3
"""C. Dux — vertical (dodo-style) stand, build123d implementation.

See README.md (same directory) for the posture / mounting / stability
rationale. The default export is STL for
slicing; pass --step to also write STEP, the preferred format for outsourced
3D printing (JLCPCB takes exact B-rep instead of faceted meshes).

Highlights over the retired OpenSCAD version:
  * All PCB data (M4 hole positions, board outline) is parsed directly from
    the KiCad PCB file at runtime, so a board revision can never silently
    desync the stand — the script asserts against the last hand-verified
    table and refuses to run if anything moved.
  * B-rep fillets: the base slab gets a rounded top perimeter, which CSG
    mesh tools cannot do.
  * Demountable two-part design: the base slab is printed ONCE (it is
    symmetric and shared by both sides and every tilt angle); the upright
    (backplate + posts + boot foot) is printed per tilt angle. The joint is
    two shear keys plus 2x M4x12 screws driven up from the open base
    underside into nuts seated in the boot — swapping angles means
    reprinting only the upright.
  * Junction reinforcement survives the split: the full-width haunch and
    the back-face ribs moved onto the upright's boot.
  * Single mounting set, minimal upright: one 4-post pattern (--set-angles,
    default -35 deg in-plane rotation = home-row slope), and the backplate
    only rises to just above the topmost screw hole's vertical projection
    (plus a rim) — the PCB above the top posts cantilevers on them. Above
    the rib band the outline tapers in to just envelope the post columns;
    the plate never touches the PCB, so nothing else up there is needed.

Mounting orientation (per user): the single extra pinky key on its stalk
faces DOWN, the 3-key thumb cluster faces UP.

Coordinate mapping (derived, not hard-coded — see parse_pcb); both axes
flipped vs the KiCad view per the mounting orientation above:
    u = Y_max - Y_kicad          (horizontal)
    v = X_kicad - X_min          (height, from the stalk-key edge)

Rotation set — one 4-hole pattern per upright, rotated in-plane about its
centroid by --set-angles (default -35 deg CCW in the front view; negate to
mirror the tilt direction). The solver centers the set laterally and lifts
it so the rotated board's lowest corner clears the base top by 2 mm
(rotation swings a board corner below the nominal edge).

Usage:
    uv sync                                  # once; creates .venv + uv.lock
    uv run stand.py --part base              # shared base slab (print once)
    uv run stand.py --part upright           # left upright, 30 deg recline
    uv run stand.py --side right --part upright
    uv run stand.py --part upright --tilt 10,15,20,25,30   # a batch of angles
    uv run stand.py --part full              # fused assembly preview
    uv run stand.py --part joint             # small fit-test coupons
    uv run stand.py --part fit               # legacy flat validation plate
    uv run stand.py --set-angles 20          # in-plane angle of the one set
    uv run stand.py --part base --step       # also export STEP for outsourcing
"""

from __future__ import annotations

import argparse
import contextlib
import io
import itertools
import math
import re
import sys
import warnings

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
    Rectangle,
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

# --- geometry parameters (also see README) ------------------------------------
plate_t = 6
plate_margin = 10
bottom_ext = 6.5  # plate below the PCB bottom edge; the solver's v_floor
#                  keeps the board's lowest corner clear of the base top
top_rim = 5  # vertical (world-z) plate rim above the topmost post top: the
#              backplate spans only the screw holes' desk projection (user
#              sizing rule); PCB above the top posts cantilevers on them
plate_r = 8
post_d = 12
post_edge_margin = 8  # plate edge beyond a post center: post radius plus
#                      pocket/wall; also the keep-out used by the taper
#                      outline (_taper_outline)
taper_margin = 2  # the full-width band extends this far above the rib tops
#                  before the outline tapers in to the post columns
nut_af = 7.4
nut_h = 3.4
screw_d = 4.5
tap_d = 3.4
base_t = 6  # the seat plane: base top = upright's boot bottom. 8 was sized
#            for walls around the (since deleted) coin slots; 6 is plenty
#            for a flat slab and drops the keyboard 2 mm
base_front = 30  # apron towards the user, trimmed for cost: typing loads
#                 lean on the rear lever arm, the front pads carry little
base_rear = 100  # rearward depth on the tilt side: longer tip-over lever arm
base_side = 10  # lateral margin past the PCB: 10 makes the base (132.5 mm)
#                exactly flush with the upright (plate 132.5 wide); the old 16
#                predates a PCB/plate change and left 6 mm of dead slab per
#                side — lateral stance rides on the pad spread
base_r = 10  # base slab corner radius (plan view)
# Junction reinforcement — vertical typing hammers the base-upright junction;
# the junction itself is now a demountable joint (constants below).
haunch_d = 18  # full-width haunch: depth along the seat plane
haunch_h = 10  # haunch height above the seat plane: a shallow ramp that
#               tops out just above the boot, keeping the boot ledge around
#               the nut pockets open (the taller reinforcement is the ribs')
rib_t = 12  # rib thickness (X); rib x-positions are solved automatically to
#            clear every post AND the joint hardware; widened 8 -> 12
#            because the 30/-35 post layout only fits two ribs
rib_h = 70  # rib height up the back face (was 60)
pad_d = 20
pad_fit = 0.2  # radial install clearance at the pocket mouth (0.4 on dia,
#                same baseline as joint_clear)
pad_recess = 0.8  # pocket depth ≈ pad thickness, pad face ends up flush
pad_seat_w = 3.0  # flat seat annulus at full depth; inboard of it the floor
#                   rises at 45° to a flush center. Face-down, only this 3 mm
#                   annulus bridges — a full flat floor is a suspended Ø20
#                   bridge and sagged badly in the first print. The wall +
#                   seat band keep the pad edge captured so it cannot creep.
pad_edge = 2  # pad circle edge to the slab straight edge; pads sit as far out
#              as this allows — the tip-over pivots are the PAD contact lines,
#              so every mm outward is a mm of tip lever (was 4: keep-out, not
#              stability; 2 mm still leaves the 0.8 mm recess fully on flat)
base_top_fillet = 2  # B-rep extra: round-over on the base top perimeter

# --- base ↔ upright joint -------------------------------------------------------
# The stand ships as two printed parts: the base slab (printed once — shared
# by every tilt angle and both sides) and a per-angle upright (backplate +
# posts + boot foot). Every joint feature is laid out in WORLD coordinates on
# the horizontal seat plane z = base_t, so one base accepts uprights of any
# tilt. Assembly per side: drop the upright onto the two keys, then drive
# 2x M4x12 socket screws UP from the open base underside into nut pockets
# seated in the boot top — screwing down from the boot ledge would put the
# driver under the reclined plate. The seat plane carries compression, the
# keys carry shear and location, the screws carry lift and clamp.
TILT = 30.0  # stand recline from vertical, deg — per-upright (--tilt);
#             current working default: 30 deg with SET_ANGLES [-35]
joint_clear = 0.4  # mating clearance per face (README tolerance baseline)
key_l = 30  # shear key footprint: key_l along X, key_w along Y, key_h tall
key_w = 8
key_h = 3.2  # shorter than the boot is tall: the boot seats on the base
#             top plane, never bottoming out on the keys
key_x = 12  # key centerline |x|; keys span y ∈ (key_y0, key_y1)
key_y0, key_y1 = 12, 20
key_chamfer = 0.5  # 45° lead-in on the key top edges
boot_h = 8  # boot foot height; its top ledge hosts the nut pockets. 8 puts
#            4.4 mm of wall over the pockets and stiffens the junction band
boot_front = -2  # boot front edge — behind the plate front face at the seat
boot_rear = 34  # boot rear edge: 6 mm past the rib wedge footprints (base
#                 y = 28) so the ribs keep their 6 mm wall, and ~15 mm of
#                 wall behind the nut seats; extended 8 mm rearward (was
#                 26) to put the printed part's CoM back inside the bed
#                 contact patch (t30/-35 CoM y 29.7 was 3.7 mm outside)
lip_chamfer = 45  # 45 deg chamfer leg (mm) across the plate's lower front
#                   lip; 0 disables. Anchored at the plate's seat-plane
#                   corner (y=0, z=base_t) so the 45 deg face bounds the
#                   print from layer 1: the raw bottom edge face sweeps
#                   over the bed at (90 - tilt) from horizontal - in front
#                   of the boot it printed as a floating ramp (0.55
#                   mm/layer at 20 deg); at 45 deg each layer advances
#                   <= half a line width, so the ramp is float-free
flange_t = 0.6  # breakaway flange thickness (2 layers, --flange): stiff
#                 enough to anchor tall prints on slick beds, snaps off
#                 with pliers after printing
joint_screw_d = 4.7  # M4 clearance through boot + base; 4.5 printed undersize
#                     and oval (FDM hole shrink) — +0.2 keeps the screw free
joint_screw_x = 58  # on the boot ledge for every tilt in 10..40 deg; must
#                    clear the keys and the ribs (see the rib keep-outs)
joint_screw_y = 14.5
# Screw direction: the nuts live in the BOOT, the screws come up from the
# open base underside. The first draft drove them down from the boot top,
# but the reclined plate overhangs that ledge (~48 mm up at 20 deg, more at
# higher tilts) and
# leaves no room for a driver; from below the access is unobstructed, and
# a longer screw costs nothing.
joint_screw = 'M4x12'  # informational — printed in the BOM note, not geometry
head_pocket_d = 8.0  # socket-head pocket in the base underside: the heads
#                     must sit below the desk face or the base would rock
head_pocket_h = 4.2
boot_nut_clear = 3.6  # boot material between its bottom and the nut floor:
#                      this pocket floor takes the clamp load — keep it
#                      chunky, plastic creeps under a sustained bolt load
nut_mouth_w = 6.9  # nut-pocket mouth pinched below the nut's 7.0 across
#                   flats: push the nut in past the flexing lip and it
#                   stays with the upright
nut_pocket_ac = 8.6  # hex pocket across corners + clearance (rib keep-out)

# --- rotation set ---------------------------------------------------------------
# A single 4-post set per upright; its in-plane rotation (--set-angles,
# default 20 deg CCW in the front view, negate to mirror) sets the home-row
# slope. The solver centers the set laterally and lifts it so the rotated
# board's lowest corner clears the base top (BASE_TOP_MARGIN).
SET_ANGLES = [-35]  # deg, CCW in front view (negate to mirror tilt)
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
        if any(abs(g - e) > HOLE_TOL for g, e in zip(got, expected, strict=True)):
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
    joint_screw_head_pockets on the underside)."""
    bores = None
    for sx in (-1, 1):
        bore = Pos(sx * joint_screw_x, joint_screw_y, -EPS) * Cylinder(
            joint_screw_d / 2,
            base_t + 2 * EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        bores = bore if bores is None else bores + bore
    return bores


def joint_screw_head_pockets():
    """Socket-head pockets open at the base underside: the heads must sit
    below the desk face or the base would rock on them, and the driver
    works from the open bottom anyway.

    The shoulder is a 45° cone (head_pocket_d at head_pocket_h, rising to
    the clearance bore) rather than a flat annulus: face-down, a flat
    shoulder prints as a suspended Ø8 bridge — saggy on the exact surface
    the head clamps against. The cone prints layer-supported and the Ø7
    head bears on the cone's Ø7 ring; clamp stays axial."""
    cone_h = (head_pocket_d - joint_screw_d) / 2
    assert head_pocket_h + cone_h < base_t, (
        'head pocket + shoulder cone must stay inside base_t'
    )
    pockets = None
    for sx in (-1, 1):
        pocket = Pos(sx * joint_screw_x, joint_screw_y, -EPS) * Cylinder(
            head_pocket_d / 2,
            head_pocket_h + EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        pocket += Pos(
            sx * joint_screw_x, joint_screw_y, head_pocket_h - EPS
        ) * Cone(
            head_pocket_d / 2,
            joint_screw_d / 2,
            cone_h + EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def joint_nut_pockets():
    """Boot-side nut seats, one negative per screw: clearance from the boot
    bottom up to the nut floor, a hex pocket with nut_h (0.2 mm) of
    headroom, and a pinched mouth at the boot top so the nut snaps in and
    stays with the upright — each angle's upright can keep its own pair."""
    hex_r = (nut_af / 2) / math.cos(math.radians(30))
    floor = boot_nut_clear  # above the boot bottom
    pockets = None
    for sx in (-1, 1):
        pocket = Pos(sx * joint_screw_x, joint_screw_y, base_t - EPS) * (
            Cylinder(
                joint_screw_d / 2,
                floor + 2 * EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
            + Pos(0, 0, floor)
            * extrude(RegularPolygon(hex_r, 6, major_radius=True), nut_h)
            + Pos(0, 0, floor + nut_h)
            * Box(
                nut_mouth_w,
                nut_mouth_w,
                boot_h - floor - nut_h + EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def joint_grooves():
    """Key pockets in the boot underside, cut all the way through. The
    joint_clear margin over the keys keeps the boot seated on the base top
    plane, not on the keys; within key height the pocket walls are the shear
    path either way, and the seat-plane bearing footprint is unchanged. The
    open window also prints clean boot-down — the old blind-pocket roof was
    a one-side-anchored 8.8 mm bridge for zero structural return."""
    grooves = None
    for sx in (-1, 1):
        groove = Pos(
            sx * key_x - key_l / 2 - joint_clear,
            key_y0 - joint_clear,
            base_t - EPS,
        ) * Box(
            key_l + 2 * joint_clear,
            key_w + 2 * joint_clear,
            boot_h + 2 * EPS,
            align=(MIN, MIN, MIN),
        )
        grooves = groove if grooves is None else grooves + groove
    return grooves


def _coupon_box(z0: float, height: float):
    """Common footprint for the two joint-coupon fragments: wide enough to
    hold a key and a screw position at their production spacing."""
    frag_y0, frag_y1 = key_y0 - 18, key_y1 + 6
    return Pos(0, (frag_y0 + frag_y1) / 2, z0 + height / 2) * Box(
        2 * (joint_screw_x + 12), frag_y1 - frag_y0, height
    )


def joint_coupon_base():
    """Fit-test coupon, base side: key + screw bore + head pocket, printed
    flat. Checks the 0.4 mm key clearance and the head-pocket depth."""
    c = _coupon_box(0, base_t)
    c += joint_keys()
    c -= joint_screw_bores() + joint_screw_head_pockets()
    return c


def joint_coupon_boot():
    """Fit-test coupon, boot side: groove + nut seat (floor, hex pocket,
    pinched mouth), printed flat on what would be the seat plane. Checks
    the groove fit and whether the nut snaps past the lip and stays."""
    c = _coupon_box(base_t, boot_h)
    c -= joint_grooves() + joint_nut_pockets()
    return Pos(0, 0, -base_t) * c  # onto the bed


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def _convex_hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain: strictly convex hull, CCW, no collinear pts."""
    pts = sorted(set(pts))
    if len(pts) < 3:
        return []

    def half(seq: list[tuple[float, float]]) -> list[tuple[float, float]]:
        out: list[tuple[float, float]] = []
        for p in seq:
            while len(out) >= 2 and (
                (out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])
                <= 0
            ):
                out.pop()
            out.append(p)
        return out

    return half(pts)[:-1] + half(pts[::-1])[:-1]


def _ccw(pts):
    """Point list normalized to counter-clockwise winding."""
    area = sum(
        pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
        for i in range(len(pts))
    )
    return pts if area > 0 else pts[::-1]


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
        flange_w: float = 0.0,
    ):
        self.pcb_w, self.pcb_h, self.holes = pcb_w, pcb_h, holes
        self.tilt = tilt
        self.post_h = post_h
        self.attach = attach
        self.flange_w = flange_w

        self.origin_z = base_t  # the plate rests on the seat plane; the boot
        #                         foot + keys + joint screws carry the junction
        self.bx = pcb_w + 2 * base_side
        self.y0 = -base_front
        self.y1 = base_rear
        self.ct, self.st = math.cos(math.radians(tilt)), math.sin(math.radians(tilt))

        self.sets, self.set_meta = self._solve_sets(set_angles)
        self.all_posts = [(x, y) for _, pts in self.sets for x, y in pts]
        # Height rule (user): the upright only needs to cover, vertically, from
        # the desk up to the topmost screw hole — the plate top edge clears the
        # topmost post top (nut-pocket opening) by top_rim of world z; the PCB
        # above the top posts cantilevers on the posts.
        top_post_y = max(y for _, y in self.all_posts)
        self.plate_h = (
            top_post_y
            + (plate_t + self.post_h) * self.st / self.ct
            + top_rim / self.ct
        )
        # plate must cover every post with room for the radius + wall margin
        keep = post_d / 2 + post_edge_margin
        self.plate_half = max(
            pcb_w / 2 + plate_margin, max(abs(x) for x, _ in self.all_posts) + keep
        )
        self.plate_w = 2 * self.plate_half
        # the bottom band stays flush with the base; a post whose keep-box
        # pokes past the base width keeps a local bulge instead of widening
        # the whole part
        self.bulges: list[tuple[int, list[tuple[float, float]]]] = []
        for sx in (-1, 1):
            spans = sorted(
                (y - keep, min(y + keep, self.plate_h))
                for x, y in self.all_posts
                if (x > 0) == (sx > 0) and abs(x) + keep > self.bx / 2
            )
            merged: list[list[float]] = []
            for lo, hi in spans:
                if merged and lo <= merged[-1][1]:
                    merged[-1][1] = max(merged[-1][1], hi)
                else:
                    merged.append([lo, hi])
            if merged:
                self.bulges.append((sx, [tuple(m) for m in merged]))
        if self.plate_half > self.bx / 2:
            spans_txt = ' / '.join(
                f'side {sx:+d} y '
                + ', '.join(f'{lo:.0f}..{hi:.0f}' for lo, hi in spans)
                for sx, spans in self.bulges
            )
            print(
                f'note: bottom band flush with the base ({self.bx:.1f} mm); '
                f'post bulge(s) kept at {spans_txt}'
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
        then pack — each set slides laterally (brute force over a shift grid,
        also what centers a single set) and rises so that the rotated board's
        lowest corner clears the base top and all posts keep MIN_POST_CENTER
        apart. Optimizes for the largest minimum post distance, tie-broken by
        a compact span and a centered shift."""
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
                for (_, _, pts), s in zip(lifted, combo, strict=True)
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
            key = (
                min(dmin, 20.0),
                -(max(combo) - min(combo)),
                -max(abs(s) for s in combo),  # prefer a centered set
            )
            if best_key is None or key > best_key:
                best, best_key = combo, key
        if best is None:
            sys.exit(
                'error: no post arrangement fits these rotation angles — '
                'try fewer sets or smaller steps (--set-angles)'
            )
        out = []
        meta = []
        for (ang, lift, pts), s in zip(lifted, best, strict=True):
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
            if abs(abs(x) - joint_screw_x) < nut_pocket_ac / 2 + rib_t / 2 + 1:
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
        px = self.bx / 2 - base_r - pad_edge
        fy = self.y0 + base_r + pad_edge
        ry = self.y1 - base_r - pad_edge
        return [(-px, fy), (px, fy), (0, fy), (-px, ry), (px, ry), (0, ry)]

    # -- backplate + posts, plate-local frame (x right, y up, z out front) ----

    def _taper_outline(self) -> list[tuple[float, float]]:
        """Retained outline above the full-width band: the convex hull of the
        upper posts' keep-boxes (center +/- post_d/2 + post_edge_margin, top
        clipped to the plate) plus the band's corners at the taper start. The
        plate never touches the PCB, so up there it only carries the post
        columns and the web between them — the hull is exactly that. Returns
        [] when the band already reaches the plate top."""
        y0 = rib_h / self.ct + taper_margin
        if y0 >= self.plate_h:
            return []
        keep = post_d / 2 + post_edge_margin
        # hull bottom corners at the base width: the bottom band is flush;
        # post keep-boxes past it widen the hull locally (the bulge)
        pts = [(-self.bx / 2, y0), (self.bx / 2, y0)]
        for x, y in self.all_posts:
            if y + keep <= y0:
                continue  # post stays inside the full-width band
            pts += [
                (x - keep, max(y - keep, y0)),
                (x + keep, max(y - keep, y0)),
                (x - keep, min(y + keep, self.plate_h)),
                (x + keep, min(y + keep, self.plate_h)),
            ]
        return _convex_hull(pts)

    def plate_and_posts(self):
        solid = Pos(0, self.plate_h / 2, 0) * extrude(
            RectangleRounded(self.plate_w, self.plate_h, plate_r), plate_t
        )
        # Taper the outline above the rib band: drop everything outside the
        # post columns' hull (cutter = band-above-the-taper minus hull).
        if hull := self._taper_outline():
            y0 = rib_h / self.ct + taper_margin
            band = Pos(-self.plate_half, y0) * Rectangle(
                self.plate_w, self.plate_h - y0, align=(Align.MIN, Align.MIN)
            )
            cutter = band - Polygon(*hull, align=None)
            solid -= Pos(0, 0, -EPS) * extrude(cutter, plate_t + 2 * EPS)
        # flush sides: cap the plate at the base width; on a side whose post
        # keep-box pokes past it, the tab grows out of the flush edge through
        # tangent quarter-round shoulders (clear of the post keep circle)
        over = self.plate_half - self.bx / 2
        for sx in (-1, 1):
            spans = dict(self.bulges).get(sx, [])
            x_mid = sx * (self.bx / 2 + over / 2)
            if not spans:
                if over > EPS:  # exactly base-flush: nothing past the edge
                    solid -= Pos(x_mid, self.plate_h / 2, -EPS) * Box(
                        over, self.plate_h, plate_t + 2 * EPS,
                        align=(Align.CENTER, Align.CENTER, MIN),
                    )
                continue
            lo0, hi0 = spans[0]
            x_in, x_out = self.bx / 2, self.plate_half
            pts = [(x_in, 0.0), (x_in, lo0 - over)]
            pts += [
                (x_in + sx * over * math.cos(math.radians(t)), lo0 + over * math.sin(math.radians(t)))
                for t in range(270 + 15, 361, 15)
            ]
            pts += [(x_out, hi0)]
            pts += [
                (x_in + sx * over * math.cos(math.radians(t)), hi0 + over * math.sin(math.radians(t)))
                for t in range(15, 91, 15)
            ]
            pts += [(x_in, self.plate_h), (x_out, self.plate_h), (x_out, 0.0)]
            if sx < 0:
                pts = [(-x, y) for x, y in reversed(pts)]
            solid -= Pos(0, 0, -EPS) * extrude(
                Polygon(*_ccw(pts), align=None), plate_t + 2 * EPS
            )
            for (_, hi_a), (_, lo_b) in itertools.pairwise(spans):
                solid -= Pos(x_mid, (hi_a + lo_b) / 2, -EPS) * Box(
                    over, lo_b - hi_a, plate_t + 2 * EPS,
                    align=(Align.CENTER, Align.CENTER, MIN),
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
            # Pad pocket: vertical wall at pad_d + 2*pad_fit captures the pad
            # edge (no creep), pad_seat_w flat annulus is the seat, then a
            # 45° cone up to the flush center — face-down, only the seat
            # annulus bridges (the first print's full-flat Ø20 floor sagged).
            ro = pad_d / 2 + pad_fit
            ri = ro - pad_seat_w
            pocket = Pos(px, py, -EPS) * Cylinder(
                ro, pad_recess + EPS, align=(Align.CENTER, Align.CENTER, MIN)
            ) - Pos(px, py, -EPS) * Cylinder(
                ri, pad_recess + 3 * EPS, align=(Align.CENTER, Align.CENTER, MIN)
            )
            pocket += Pos(px, py, -EPS) * Cone(
                ri + EPS,
                ri - pad_recess,
                pad_recess + EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
            base -= pocket
        base += joint_keys()
        base -= joint_screw_bores() + joint_screw_head_pockets()
        return base

    # -- upright: backplate + posts + boot foot, one part per tilt angle ------

    def upright_solid(self):
        """The demountable half: everything above the seat plane, printed
        boot-down on the bed. Exported pose is the print pose; layer-by-layer
        audits (facets + per-layer float) show no bridging and no air-start
        once the front lip is chamfered."""
        rad = math.radians(self.tilt)
        body = (
            Pos(0, 0, self.origin_z)
            * Rot(X=90 - self.tilt)
            * self.plate_and_posts()
        )
        if lip_chamfer > 0:
            # Cut the plate's lower front lip at 45 deg (see lip_chamfer).
            # Applied before the boot union: the cut plane starts at the
            # plate's seat-plane corner, inside the boot footprint, so the
            # boot itself is untouched.
            body -= (
                Pos(-(self.plate_w + 20) / 2, 0, 0)
                * extrude(
                    Plane.YZ
                    * Polygon(
                        (0, base_t),
                        (-lip_chamfer, base_t + lip_chamfer),
                        (-lip_chamfer, base_t),
                        align=None,
                    ),
                    self.plate_w + 20,
                )
            )
        # boot + haunch stay flush with the base even when the plate keeps a
        # post bulge further up
        flush_w = min(self.plate_w, self.bx)
        boot = Pos(0, (boot_front + boot_rear) / 2, base_t + boot_h / 2) * Box(
            flush_w, boot_rear - boot_front, boot_h
        )
        # Full-width haunch: a shallow ramp up the back face that tops out
        # just above the boot (haunch_h), filling the junction inner corner.
        boot += Pos(-flush_w / 2, boot_front, base_t) * extrude(
            Plane.YZ * self._wedge(haunch_d, self.haunch_h, rad), flush_w
        )
        # Ribs continue up the back face into the hammering zone. Wedge
        # base depth rib_t + 18: base reaches y = 28, keeping the 6 mm
        # wall to the extended boot rear (34).
        for rx in self.rib_x:
            boot += Pos(rx - rib_t / 2, boot_front, base_t) * extrude(
                Plane.YZ * self._wedge(rib_t + 18, rib_h, rad), rib_t
            )
        body += boot
        if self.flange_w > 0:
            body += self._breakaway_flange(flush_w)
        body -= joint_grooves()
        body -= joint_nut_pockets()
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

    def _breakaway_flange(self, flush_w: float):
        """Thin ring around the boot footprint (--flange): bed-adhesion
        insurance for slick plates. flange_t tall so it snaps off with
        pliers after printing; added after the lip chamfer cut, so the two
        never interact, and clear of every joint feature."""
        outer = RectangleRounded(
            flush_w + 2 * self.flange_w,
            boot_rear - boot_front + 2 * self.flange_w,
            4,
        )
        inner = Rectangle(flush_w, boot_rear - boot_front)
        return Pos(0, (boot_front + boot_rear) / 2, base_t) * extrude(
            outer - inner, flange_t
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
    """Send the model to the OCP CAD Viewer (VS Code). Best effort: never
    fatal, never noisy — the library prints (and sometimes swallows) its own
    connection errors, so its output is captured and condensed into one line
    when the viewer is closed or half-open."""
    buf_out, buf_err = io.StringIO(), io.StringIO()
    exc: Exception | None = None
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            with contextlib.redirect_stdout(buf_out), contextlib.redirect_stderr(buf_err):
                from ocp_vscode import Camera, show

                show(part, names=[name], reset_camera=Camera.CENTER)
    except ImportError:
        print('note: viewer push skipped — install with: uv add --dev ocp-vscode')
        return
    except Exception as e:
        exc = e
    if exc is None and not buf_err.getvalue().strip():
        print(f'pushed to OCP CAD Viewer: {name}')
    else:
        detail = buf_err.getvalue().strip().splitlines() or [str(exc) or 'unknown error']
        print(f'note: viewer push skipped ({detail[0][:100]})')


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
        '--flange',
        type=float,
        default=0.0,
        metavar='W',
        help='add a breakaway flange around the boot for slick beds: W mm '
        'out, 0.6 mm thick, snap off after printing (0 = off)',
    )
    ap.add_argument(
        '--pcb',
        type=Path,
        default=DEFAULT_PCB,
        help='KiCad PCB file to parse hole positions from',
    )
    ap.add_argument('--out', type=Path, default=Path(__file__).parent / 'out')
    ap.add_argument(
        '--step',
        action='store_true',
        help='also export STEP (exact B-rep, for outsourced 3D printing)',
    )
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
    if not 0 <= args.flange <= 30:
        sys.exit('error: --flange width must be 0..30 mm')

    pcb_w, pcb_h, holes = parse_pcb(args.pcb)
    args.out.mkdir(parents=True, exist_ok=True)

    def emit(part, stem: str) -> None:
        summary(stem, part)
        if args.show:
            push_viewer(part, stem)
            return
        stl = args.out / f'{stem}.stl'
        export_stl(part, stl)
        print(f'exported: {stl}')
        if args.step:
            step = args.out / f'{stem}.step'
            export_step(part, step)
            print(f'          {step}')

    if args.part == 'joint':
        emit(joint_coupon_base(), 'joint_coupon_base')
        emit(joint_coupon_boot(), 'joint_coupon_boot')
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
                Stand(
                    pcb_w,
                    pcb_h,
                    holes,
                    args.post_h,
                    args.attach,
                    set_angles,
                    tilt,
                    flange_w=args.flange,
                )
            )
            tag = f'_t{tilt:g}'
            if args.flange:
                tag += f'_flange{args.flange:g}'
            if args.part == 'upright':
                stems.append(f'stand_{args.side}_upright{tag}')
            elif args.part == 'full':
                stems.append(f'stand_{args.side}_full{tag}')
            else:
                stems.append(f'stand_{args.side}_fit')

    for stand, stem in zip(stands, stems, strict=True):
        for (ang, pts), (_, _, _, corner_z) in zip(
            stand.sets, stand.set_meta, strict=True
        ):
            print(
                f'set {ang:5.1f}°: posts x=[{min(x for x, _ in pts):7.2f}, '
                f'{max(x for x, _ in pts):7.2f}]  lowest PCB corner z={corner_z:6.2f} mm'
            )
        print(
            f'ribs at x = {[round(x, 1) for x in stand.rib_x]}'
            f'  | haunch_h = {stand.haunch_h:.1f} mm'
            f'  | joint: keys x=±{key_x:g}, screws x=±{joint_screw_x:g} '
            f'({joint_screw} from below)'
        )
        emit(stand.build(args.part, args.side), stem)


if __name__ == '__main__':
    main()
