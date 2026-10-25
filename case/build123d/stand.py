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
  * Demountable two-part design: the base slab tracks the raw post-need
    footprint (derived from the post layout); the boot is that footprint
    inset by boot_inset per side, so it seats fully on the slab's flat top
    inside the 2 mm perimeter fillet, and the plate's bottom band is
    clamped to the boot footprint — flush at the bottom corners, no
    sideways overhang.
    The upright (backplate + posts + boot foot) is printed per tilt angle;
    the slab only needs a reprint when a layout change moves the band — the
    joint screws, keys and ribs all derive from it. The joint is a merged
    key bar centered on the band plus 2x M4x12 screws at the boot's outer
    edges, driven up from the open base underside into nuts in the boot.
  * Junction reinforcement survives the split: the wide haunch and the
    back-face ribs live on the upright's boot, anchored to the backplate's
    back face (embedded 1.5 mm along the full height) instead of the boot
    front — whose +4.5 recess had dragged them 4.5 mm rearward, detaching
    them from the plate they reinforce.
  * PCB held by 4 screws: 3 M4 nuts in back-face hex pockets clamp the
    board; the lowest post is self-tapped instead (Ø tap_d pilot straight
    through post + plate — after the drop its back-face pocket would sit
    inside the boot's z-band with no insertion access, and a tapped hole
    needs none). It also gets no Ø15 flare: at this drop the flare foot
    would overhang the plate's bottom edge and be clipped into a lump —
    a plain Ø10 wall prints fine. The haunch toe now wraps the pilot's
    rear exit, giving the screw tip ~1 mm of extra plastic bite.
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
it so the rotated board's lowest outline point (real Edge.Cuts material —
the bbox corner holds no board) clears the base top by 2 mm, then shifts
the whole pattern keyboard_drop down the plate (user-measured).

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
    CenterOf,
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
bottom_ext = 6.5  # nominal plate below the PCB bottom edge; the solver's
#                  v_floor keeps the board's lowest outline point clear of
#                  the base top, and keyboard_drop eats most of that margin
keyboard_drop = 30.0  # whole mounting pattern shifted DOWN the plate by this
# much (user asked for "键盘下移 3cm"; the real-outline height floor auto-
# caps the NET drop at ~29.1 mm — the solver lifts the pattern just enough
# to hold the 2 mm base-top margin: printed lowest PCB outline point
# z = 8.00, lift ~0.9. The old 20 cap guarded the lowest post's back-face
# nut pocket vs the boot top; lowest_post_tapped = True removes it)
lowest_post_tapped = True  # lowest post gets a Ø tap_d self-tapping pilot
# instead of a nut pocket: after the drop its back-face pocket would sit
# inside the boot's z-band with no insertion access, and a tapped hole needs
# none. It also gets no Ø15 flare: at this drop the flare foot would
# overhang the plate's bottom edge and be clipped into a lump
top_rim = 5  # vertical (world-z) plate rim above the topmost post top: the
#              backplate spans only the screw holes' desk projection (user
#              sizing rule); PCB above the top posts cantilevers on them
plate_r = 8
post_d = 10  # solid standoff boss; the Ø4.5 screw bore leaves a 2.75 mm wall
post_edge_margin = 5  # plate edge beyond a post center: keep = post_d/2 +
#                      post_edge_margin = 10 still clears the Ø8.6 nut
#                      pocket + lip on the back face; also the keep-out used
#                      by the taper outline (_taper_outline) and the
#                      per-side bottom-band derivation (__init__)
taper_margin = 2  # the full-width band extends this far above the rib tops
#                  before the outline tapers in to the post columns
nut_af = 7.4
nut_h = 3.4
nut_mouth_h = 1.2  # PCB nut pocket: pinch-lip depth at the plate back face
nut_pocket_depth = 4.6  # PCB nut pocket depth into the plate: lip + nut_h;
#                         leaves plate_t - 4.6 = 1.4 mm of front wall that
#                         the Ø10 post directly buttresses — the nut bears
#                         on that wall (clamp load: nut -> wall -> post)
screw_d = 4.5
tap_d = 3.4
base_t = 6  # the seat plane: base top = upright's boot bottom. 8 was sized
#            for walls around the (since deleted) coin slots; 6 is plenty
#            for a flat slab and drops the keyboard 2 mm
base_front = 30  # apron towards the user, trimmed for cost: typing loads
#                 lean on the rear lever arm, the front pads carry little
base_rear = 100  # rearward depth on the tilt side: longer tip-over lever arm
base_r = 10  # base slab corner radius (plan view)
# Junction reinforcement — vertical typing hammers the base-upright junction;
# the junction itself is now a demountable joint (constants below).
haunch_d = 12  # full-width haunch: depth along the seat plane (was 18;
#               slimmed after the first print — 4-perimeter walls suffice)
haunch_h = 10  # haunch height above the seat plane: a shallow ramp that
#               tops out just above the boot, keeping the boot ledge around
#               the nut pockets open (the taller reinforcement is the ribs')
rib_t = 12  # rib thickness (X); rib x-positions are solved automatically to
#            clear every post AND the joint hardware; widened 8 -> 12
#            because the 30/-35 post layout only fits two ribs
rib_h = 70  # rib height up the back face (was 60)
pad_d = 20
pad_fit = 0.1  # radial install clearance at the pocket mouth (0.2 on dia;
#                separate from the joint_clear key fit below)
pad_t = 1.0  # purchased self-adhesive TPU pad thickness
pad_protrude = 0.4  # pad face above the desk face: keeps the pads the ONLY
#                     desk contact (the tip-over pivots) across die-cut
#                     thickness tolerance — TPU compression at keyboard
#                     loads is microns, the tolerance is the driver
pad_recess = pad_t - pad_protrude  # 0.6 pocket depth
pad_ring_w = 5  # flat glue annulus at pad_recess; the centre is relieved
#                pad_relief deeper, so droop/fall strands from the recess
#                ceiling land in the relief, never under the pad's glue ring
pad_relief = 1.0  # centre-recess headroom below the glue ring plane: the
#                  recess ceiling is a short Ø10.2 bridge (was Ø20.4) and
#                  its strands stop 1 mm short of the pad back
pad_edge = 2  # pad circle edge to the slab straight edge; pads sit as far out
#              as this allows — the tip-over pivots are the PAD contact lines,
#              so every mm outward is a mm of tip lever (was 4: keep-out, not
#              stability; 2 mm still leaves the 0.6 mm recess fully on flat)
base_top_fillet = 2  # B-rep extra: round-over on the base top perimeter
boot_inset = base_top_fillet  # boot setback per side: sit fully on the base
#                              slab's flat top, inside the 2 mm perimeter
#                              fillet; also the plate's bottom band edge —
#                              the band is clamped flush to the boot (no
#                              sideways overhang at the bottom corners)

# --- base ↔ upright joint -------------------------------------------------------
# The stand ships as two printed parts: the base slab (shared by every tilt
# angle while the post layout — and with it the band footprint and the screw
# bores — stays put) and a per-angle upright (backplate +
# posts + boot foot). Every joint feature is laid out in WORLD coordinates on
# the horizontal seat plane z = base_t, so one base accepts uprights of any
# tilt. Assembly per side: drop the upright onto the two keys, then drive
# 2x M4x12 socket screws UP from the open base underside into nut pockets
# seated in the boot top — screwing down from the boot ledge would put the
# driver under the reclined plate. The seat plane carries compression, the
# keys carry shear and location, the screws carry lift and clamp.
TILT = 30.0  # stand recline from vertical, deg — per-upright (--tilt);
#             current working default: 30 deg with SET_ANGLES [-35]
joint_clear = 0.2  # key/groove clearance per face: snug FDM slide fit
#                    (halved from the old 0.4 baseline — less slop on the
#                    merged key bar; print a --part joint coupon FIRST to
#                    verify the tighter fit before ordering full parts)
key_l = 30  # shear key footprint: key_l along X, key_w along Y, key_h tall
key_w = 8
key_h = 3.2  # shorter than the boot is tall: the boot seats on the base
#             top plane, never bottoming out on the keys
key_x = 12  # key centerline offset from the BAND MID (band_mid ± key_x —
#            the band is asymmetric, so mid-boot anchoring keeps the merged
#            key bar centered); keys span y ∈ (key_y0, key_y1)
key_y0, key_y1 = 12, 20
key_chamfer = 0.5  # 45° lead-in on the key top edges
boot_h = 8  # boot foot height; its top ledge hosts the nut pockets. 8 puts
#            4.4 mm of wall over the pockets and stiffens the junction band
boot_front = 0  # boot front edge flush with the plate's back face at the
#                seat: no front lip AND no step — the plate's back face
#                meets the boot front continuously (the old +4.5 recess
#                left a triangular sliver gap/step at the junction). Only
#                the boot box tracks this constant — the reinforcement
#                wedges anchor to the back face instead (see _wedge), so
#                moving the front cannot detach them; the lowest post's
#                pilot exit is buried in boot/toe plastic and re-cut
#                through it (extra bite for the self-tapper)
boot_rear = 34  # boot rear edge DEFAULT (floor): each Stand derives its real
#                 rear in __init__ from three floors — wall behind the nut
#                 pockets, 6 mm behind the (slimmed) rib wedge feet, and the
#                 free-standing CoM margin — and extends rearward past this
#                 default whenever the measured CoM demands it
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
joint_screw_d = 4.5  # M4 clearance through boot + base — the printed and
#                     assembled reference; 31bef39's +0.2 oversize never
#                     printed and was reverted with the rest of it
# joint screw |x| is DERIVED per layout (see Stand.__init__): the two
# screws hug the boot's outer edges — their nut seats must sit on the
# narrowed boot, and the boot edge is still the widest possible lever
# against pull-out — leaving the keys centered on the boot in between.
# History: ±58 on the old full-width boot, then ±35 inboard once the boot
# matched the band; now boot_edge ∓ (nut pocket + 1.5 mm wall).
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
nut_mouth_af = 6.9  # nut-pocket mouth, a hex pinch below the nut's 7.0
#                    across flats: push the nut in past the flexing lip
#                    and it stays with the upright. The pinch MUST be a
#                    hex aligned with the pocket: a square mouth needs a
#                    ~7.25 side to pass a 7.0 hex nut in-plane, so a
#                    square opening this size just blocks the nut.
nut_pocket_ac = 8.6  # hex pocket across corners + clearance (rib keep-out)

# --- rotation set ---------------------------------------------------------------
# A single 4-post set per upright; its in-plane rotation (--set-angles,
# default 20 deg CCW in the front view, negate to mirror) sets the home-row
# slope. The solver centers the set laterally and lifts it so the rotated
# board's lowest outline point clears the base top (BASE_TOP_MARGIN), then
# shifts the whole pattern keyboard_drop down the plate.
SET_ANGLES = [-35]  # deg, CCW in front view (negate to mirror tilt)
MIN_POST_CENTER = 11.0  # post diameter 10 + 1 mm printing clearance
SHIFT_GRID = 3.0  # lateral shift candidates per set, packed by brute force
SHIFT_SPAN = 45.0  # max |lateral shift| of a set centroid
PLATE_HALF_BOUND = 72.0  # lateral layout sanity cap: the solver keeps post
#                          |x| <= this - 8, and the bottom band (hence the
#                          base slab — the band IS the slab now) never
#                          exceeds it either; a wider wish is a layout
#                          problem, not a wider slab
BASE_TOP_MARGIN = 2.0  # rotated board's lowest point vs base top — the ONLY
#                        height constraint: clear the base, sit as low as
#                        possible (the old 43 mm bottom-line rule applied only
#                        to a base-parallel, unrotated mounting)
EPS = 0.01
MIN = Align.MIN


# ---------------------------------------------------------------------------
# PCB parsing — single source of truth for hole positions and board outline
# ---------------------------------------------------------------------------


def parse_pcb(
    path: Path,
) -> tuple[float, float, list[tuple[float, float]], list[tuple[float, float]]]:
    """Return (pcb_w, pcb_h, [(u, v) x4], outline) from a KiCad PCB file.

    outline is the real Edge.Cuts boundary sampled at ~1 mm steps in the
    stand frame — the solver's height constraint uses it instead of the
    bounding box, whose corner (u = pcb_w, v = 0) holds no material.
    """
    text = path.read_text()

    # Board outline: ergogen exports Edge.Cuts as gr_line/gr_arc segments
    # (arcs tessellated into short lines, so endpoints suffice for the bbox).
    xs: list[float] = []
    ys: list[float] = []
    segs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for line in text.splitlines():
        if 'Edge.Cuts' not in line:  # KiCad writes the layer name unquoted
            continue
        if not line.strip().startswith('(gr_'):
            continue
        # start/end/center only: gr_arc "mid" points are corner-rounding
        # bulges outside the straight-edge hull, and the stand frame (like
        # the verified hole table) is anchored to the straight-edge corners.
        xy = [
            (float(a), float(b))
            for a, b in re.findall(
                r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line
            )
        ]
        for a, b in xy:
            xs.append(a)
            ys.append(b)
        # first two tokens are (start, end) — center/mid come later in the
        # s-expression — so this pairs each segment's endpoints correctly
        if len(xy) >= 2:
            segs.append((xy[0], xy[1]))
    if not xs:
        sys.exit(f'error: no Edge.Cuts geometry found in {path}')
    x_min, y_max = min(xs), max(ys)
    pcb_h = max(xs) - x_min  # board height (along columns) runs along X in KiCad
    pcb_w = y_max - min(ys)

    # Real outline samples (stand frame): interpolate each segment at ~1 mm
    # steps. The bbox corner would overstate how low the rotated board hangs
    # — the lowest real material (pinky/ring columns' bottom edge) sits far
    # above it, and the whole mounting pattern rides that difference.
    outline: list[tuple[float, float]] = []
    for (x1, y1), (x2, y2) in segs:
        n = max(2, round(math.hypot(x2 - x1, y2 - y1)))
        for i in range(n + 1):
            f = i / n
            outline.append((y_max - (y1 + (y2 - y1) * f), x1 + (x2 - x1) * f - x_min))

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
    return pcb_w, pcb_h, ordered, outline


# ---------------------------------------------------------------------------
# Joint geometry — world frame, defined once so the base accepts uprights of
# any tilt (and so the --part joint coupons reuse the exact production
# features instead of a lookalike)
# ---------------------------------------------------------------------------


def joint_keys(band_mid: float):
    """Two shear keys standing on the seat plane, centered on the asymmetric
    bottom band (band_mid ± key_x) so they always sit mid-boot. They locate
    the upright in X/Y/yaw and carry the horizontal typing loads; the seat
    plane carries compression, the screws carry lift and clamp."""
    keys = None
    for sx in (-1, 1):
        key = Pos(band_mid + sx * key_x - key_l / 2, key_y0, base_t) * Box(
            key_l, key_w, key_h, align=(MIN, MIN, MIN)
        )
        keys = key if keys is None else keys + key
    # 45° lead-in all around the top so the boot grooves drop on cleanly
    return chamfer(keys.edges().group_by(Axis.Z)[-1], key_chamfer)


def joint_screw_bores(xs: list[float]):
    """Vertical M4 clearance holes through the base (pair with
    joint_screw_head_pockets on the underside)."""
    bores = None
    for x in xs:
        bore = Pos(x, joint_screw_y, -EPS) * Cylinder(
            joint_screw_d / 2,
            base_t + 2 * EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        bores = bore if bores is None else bores + bore
    return bores


def joint_screw_head_pockets(xs: list[float]):
    """Socket-head pockets open at the base underside: the heads must sit
    below the desk face or the base would rock on them, and the driver
    works from the open bottom anyway.

    Flat annulus shoulder, the printed-and-assembled reference: face-down
    it is a ring bridge (Ø8 to the Ø4.5 bore, ~1.75 mm wide) that printed
    fine and seats the head parallel to the desk. 31bef39's 45° cone
    shoulder never printed; reverted with the rest of that commit's
    unprinted tweaks. Minor stringing inside the pocket is a slicer
    matter — see the README print tips."""
    pockets = None
    for x in xs:
        pocket = Pos(x, joint_screw_y, -EPS) * Cylinder(
            head_pocket_d / 2,
            head_pocket_h + EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def joint_nut_pockets(xs: list[float]):
    """Boot-side nut seats, one negative per screw: clearance from the boot
    bottom up to the nut floor, a hex pocket with nut_h (0.2 mm) of
    headroom, and a pinched mouth at the boot top so the nut snaps in and
    stays with the upright — each angle's upright can keep its own pair."""
    hex_r = (nut_af / 2) / math.cos(math.radians(30))
    mouth_r = (nut_mouth_af / 2) / math.cos(math.radians(30))
    floor = boot_nut_clear  # above the boot bottom
    pockets = None
    for x in xs:
        pocket = Pos(x, joint_screw_y, base_t - EPS) * (
            Cylinder(
                joint_screw_d / 2,
                floor + 2 * EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
            + Pos(0, 0, floor)
            * extrude(RegularPolygon(hex_r, 6, major_radius=True), nut_h)
            + Pos(0, 0, floor + nut_h)
            * extrude(
                RegularPolygon(mouth_r, 6, major_radius=True),
                boot_h - floor - nut_h + EPS,
            )
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def joint_grooves(band_mid: float):
    """Key pockets in the boot underside, cut all the way through. The
    joint_clear margin over the keys keeps the boot seated on the base top
    plane, not on the keys; within key height the pocket walls are the shear
    path either way, and the seat-plane bearing footprint is unchanged. The
    open window also prints clean boot-down — the old blind-pocket roof was
    a one-side-anchored 8.8 mm bridge for zero structural return."""
    grooves = None
    for sx in (-1, 1):
        groove = Pos(
            band_mid + sx * key_x - key_l / 2 - joint_clear,
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


def _coupon_box(stand: Stand, z0: float, height: float):
    """Common footprint for the two joint-coupon fragments: wide enough to
    hold a key and both screw positions at their production spacing."""
    frag_y0, frag_y1 = key_y0 - 18, key_y1 + 6
    half_w = (stand.joint_screw_xs[1] - stand.joint_screw_xs[0]) / 2 + 12
    return Pos(stand.band_mid, (frag_y0 + frag_y1) / 2, z0 + height / 2) * Box(
        2 * half_w, frag_y1 - frag_y0, height
    )


def joint_coupon_base(stand: Stand):
    """Fit-test coupon, base side: key + screw bore + head pocket, printed
    flat. Checks the 0.2 mm key clearance and the head-pocket depth."""
    c = _coupon_box(stand, 0, base_t)
    c += joint_keys(stand.band_mid)
    c -= joint_screw_bores(stand.joint_screw_xs)
    c -= joint_screw_head_pockets(stand.joint_screw_xs)
    return c


def joint_coupon_boot(stand: Stand):
    """Fit-test coupon, boot side: groove + nut seat (floor, hex pocket,
    pinched mouth), printed flat on what would be the seat plane. Checks
    the groove fit and whether the nut snaps past the lip and stays."""
    c = _coupon_box(stand, base_t, boot_h)
    c -= joint_grooves(stand.band_mid) + joint_nut_pockets(stand.joint_screw_xs)
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


def _half_plane_cutter(
    a: tuple[float, float],
    b: tuple[float, float],
    x_lo: float,
    x_hi: float,
    y_lo: float,
    y_hi: float,
):
    """2D cutter polygon covering everything LEFT of the directed line
    a -> b, clipped to [x_lo, x_hi] x [y_lo, y_hi]. Sutherland-Hodgman
    clipping of the rectangle against the half plane, so the boolean
    never leaves sliver faces where the cut meets the plate boundary.
    (Picking the two farthest-projection rectangle corners instead only
    works while those are adjacent corners; once both land on the same
    edge the quad folds into a concave dart and silently under-cuts —
    the new left top-corner line hit exactly that.)"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    ln = math.hypot(dx, dy)
    nx, ny = -dy / ln, dx / ln  # unit normal, left of a -> b
    rect = [(x_lo, y_hi), (x_hi, y_hi), (x_hi, y_lo), (x_lo, y_lo)]
    out: list[tuple[float, float]] = []
    for i, p in enumerate(rect):
        q = rect[(i + 1) % len(rect)]
        dp = (p[0] - a[0]) * nx + (p[1] - a[1]) * ny
        dq = (q[0] - a[0]) * nx + (q[1] - a[1]) * ny
        if dp >= 0:
            out.append(p)
        if dp * dq < 0:  # strict crossing: intersection strictly inside the edge
            t = dp / (dp - dq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    if len(out) < 3:
        sys.exit(
            f'error: half-plane cutter ({a} -> {b}) misses the clip '
            f'rectangle [{x_lo}, {x_hi}] x [{y_lo}, {y_hi}]'
        )
    return Polygon(*_ccw(out), align=None)


class Stand:
    def __init__(
        self,
        pcb_w: float,
        pcb_h: float,
        holes: list[tuple[float, float]],
        outline: list[tuple[float, float]],
        post_h: float,
        attach: str,
        set_angles: list[float],
        tilt: float = TILT,
        flange_w: float = 0.0,
    ):
        self.pcb_w, self.pcb_h, self.holes = pcb_w, pcb_h, holes
        self.outline = outline
        self.tilt = tilt
        self.post_h = post_h
        self.face_z = plate_t + post_h  # PCB seating plane over the back face
        self.attach = attach
        self.flange_w = flange_w

        self.origin_z = base_t  # the plate rests on the seat plane; the boot
        #                         foot + keys + joint screws carry the junction
        self.y0 = -base_front
        self.y1 = base_rear
        self.ct, self.st = math.cos(math.radians(tilt)), math.sin(math.radians(tilt))

        self.sets, self.set_meta = self._solve_sets(set_angles)
        self.all_posts = [(x, y) for _, pts in self.sets for x, y in pts]
        # The lowest post is self-tapped (Ø tap_d pilot, no nut): after the
        # drop its back-face pocket would sit inside the boot's z-band with
        # no insertion access, and a tapped hole needs none — with the boot
        # front recessed (+4.5) the pilot exits into open air. Nut
        # pockets (and the back-face clearances they need: haunch cap, boot
        # access window) only concern the pocket posts. Board geometry
        # (taper hull, bulges, band edges, posts themselves) keeps using
        # all_posts.
        self.lowest_post = min(self.all_posts, key=lambda p: p[1])
        self.pocket_posts = [
            p for p in self.all_posts if p != self.lowest_post
        ]
        # Height rule (user): the upright only needs to cover, vertically, from
        # the desk up to the topmost screw hole — the plate top edge clears the
        # topmost solid post top by top_rim of world z; the PCB above the top
        # posts cantilevers on the posts.
        top_post_y = max(y for _, y in self.all_posts)
        self.plate_h = (
            top_post_y + self.face_z * self.st / self.ct + top_rim / self.ct
        )
        # plate must cover every post with room for the radius + wall margin
        keep = post_d / 2 + post_edge_margin
        self.plate_half = max(
            pcb_w / 2 + plate_margin, max(abs(x) for x, _ in self.all_posts) + keep
        )
        self.plate_w = 2 * self.plate_half
        # per-side bottom-band edges (signed): POST needs only — the outmost
        # post's keep-box + 2 mm, rounded up to even mm, capped by the plate
        # blank. A side whose needs exceed the solver's lateral envelope (the
        # reworded PLATE_HALF_BOUND sanity cap) keeps the band at the cap and
        # carries the post on a local bulge tab. Ribs no longer widen the
        # band: _auto_ribs only places them where they fit INSIDE it, and a
        # rib that fits nowhere is simply skipped (fewer ribs is normal).
        cap = PLATE_HALF_BOUND - 8
        slab_edges: dict[int, float] = {}
        for sx in (-1, 1):
            post_need = max(
                (abs(x) + keep for x, _ in self.all_posts if (x > 0) == (sx > 0)),
                default=0.0,
            )
            edge = min(self.plate_half, math.ceil((post_need + 2) / 2) * 2)
            if edge > cap:
                edge = min(self.plate_half, cap)
            slab_edges[sx] = sx * edge
        self.slab_lo = slab_edges[-1]
        self.slab_hi = slab_edges[1]
        # the base slab IS the raw post-need footprint (bx = slab width, slab
        # centered on the slab's mid-x); the pads hug the slab edges too. The
        # band clamp below does not touch any of it — the slab keeps its size.
        self.bx = self.slab_hi - self.slab_lo
        # Boot footprint = slab inset by boot_inset per side: the slab's top
        # perimeter carries a base_top_fillet round-over, so only the inset
        # footprint sits fully on the flat top. Symmetric, hence
        # boot_mid == band_mid — keys and grooves stay anchored to band_mid.
        self.boot_lo = self.slab_lo + boot_inset
        self.boot_hi = self.slab_hi - boot_inset
        # The plate's bottom band is clamped to the boot footprint: the raw
        # post-need edges used to keep 2 mm of sideways overhang past the
        # narrowed boot at each bottom corner (user-flagged on a back-view
        # render), so the band now ends flush with the boot. Everything
        # downstream (taper hull corners, top-corner cut anchors, bulge test,
        # per-side plate trim) derives from band_lo/hi and follows. A post
        # whose keep-box pokes past the clamped edge still gets its bulge
        # tab (below); t30/-35 keeps H4 clear by ~0.7 mm, so only H1's tab.
        self.band_lo = max(self.slab_lo, self.boot_lo)
        self.band_hi = min(self.slab_hi, self.boot_hi)
        # per-side clamped band edges (signed) — the bulge test here and the
        # per-side plate trim in plate_and_posts key off these
        self.band_edges: dict[int, float] = {-1: self.band_lo, 1: self.band_hi}
        self.band_mid = (self.band_lo + self.band_hi) / 2
        # Joint screws hug the boot's outer edges: their nut seats must sit
        # on the narrowed boot (not out on the plate band, which ends flush
        # with the boot), and the
        # boot edge is still the widest possible lever against pull-out, with
        # the keys centered on the boot in between. Derived, so they track
        # any future band move.
        edge_need = nut_pocket_ac / 2 + 1.5  # pocket + 1.5 mm outer wall
        boot_w = self.boot_hi - self.boot_lo
        assert boot_w > 2 * edge_need, (
            f'boot {boot_w:g} mm wide cannot hold two edge screws '
            f'(need > {2 * edge_need:g} mm) — reduce the --set-angles spread'
        )
        self.joint_screw_xs = [self.boot_lo + edge_need, self.boot_hi - edge_need]
        self.rib_x = self._auto_ribs()
        if len(self.rib_x) < 2:
            print(f'warning: only {len(self.rib_x)} ribs fit inside the boot')
        # a post whose keep-box pokes past ITS side's band edge keeps a local
        # bulge tab instead of widening the whole part
        self.bulges: list[tuple[int, list[tuple[float, float]]]] = []
        for sx in (-1, 1):
            spans = sorted(
                (y - keep, min(y + keep, self.plate_h))
                for x, y in self.all_posts
                if (x > 0) == (sx > 0) and abs(x) + keep > abs(self.band_edges[sx])
            )
            merged: list[list[float]] = []
            for lo, hi in spans:
                if merged and lo <= merged[-1][1]:
                    merged[-1][1] = max(merged[-1][1], hi)
                else:
                    merged.append([lo, hi])
            if merged:
                self.bulges.append((sx, [tuple(m) for m in merged]))
        spans_txt = ' / '.join(
            f'side {sx:+d} y '
            + ', '.join(f'{lo:.0f}..{hi:.0f}' for lo, hi in spans)
            for sx, spans in self.bulges
        )
        print(
            f'note: bottom band x [{self.band_lo:g}, {self.band_hi:g}] mm '
            f'(slab [{self.slab_lo:g}, {self.slab_hi:g}]), '
            f'plate half {self.plate_half:.2f}'
            + (f'; post bulge(s) kept at {spans_txt}' if self.bulges else '')
        )
        self.haunch_h = self._cap_haunch()
        # Boot rear edge, derived per instance (module constant = floor we
        # never shrink below). Hard floors first: wall behind the nut seats,
        # 6 mm of boot behind the slimmed rib feet. Free-standing (the CoM
        # projection must keep 8 mm to the rear edge) needs the CoM, and the
        # CoM needs the boot — chicken and egg, so measure on a provisional
        # build, extend rearward until the margin holds, round UP to even mm,
        # and rebuild once. The final CoM is kept for the per-export report.
        self.boot_rear = math.ceil(
            max(
                boot_rear,  # verified default: don't go slimmer than it
                joint_screw_y + nut_pocket_ac / 2 + 1.5,  # nut-pocket wall
                (rib_t + 8) + 6,  # rib foot rear y = rib_t + 8, + 6 mm wall
            )
            / 2
        ) * 2  # even mm
        body = self.upright_solid()  # provisional pass (floors only)
        self.boot_rear = math.ceil(
            max(self.boot_rear, body.center(CenterOf.MASS).Y + 8.0) / 2
        ) * 2
        self.com_y = self.upright_solid().center(CenterOf.MASS).Y

    # -- rotation-set layout solver ------------------------------------------

    def _solve_sets(self, angles: list[float]) -> list[tuple[float, list[tuple[float, float]]]]:
        """Place one 4-hole set per angle: rotate about the pattern centroid,
        then pack — each set slides laterally (brute force over a shift grid,
        also what centers a single set) and rises so that the rotated board's
        lowest outline point clears the base top and all posts keep
        MIN_POST_CENTER apart; the whole pattern then sits keyboard_drop
        lower on the plate (user request; with lowest_post_tapped the
        outline floor is the only limit — the solver lifts the pattern
        just enough to hold it, so the NET drop is keyboard_drop minus
        that lift, ~29.1 mm at the default drop 30).
        Optimizes for the largest minimum post distance, tie-broken by
        a compact span and a centered shift."""
        cu = sum(u for u, _ in self.holes) / len(self.holes)
        cv = sum(v for _, v in self.holes) / len(self.holes)
        rel = [(u - cu, v - cv) for u, v in self.holes]
        # v' the lowest board material may reach: the plate back face under
        # it must stay BASE_TOP_MARGIN above the base top
        # face_z = plate_t + post_h: the PCB back rides that far in front of
        # the plate back face, so its world height gains face_z * st
        v_floor = (
            base_t + BASE_TOP_MARGIN - self.origin_z - self.face_z * self.st
        ) / self.ct - bottom_ext

        lifted = []
        for ang in angles:
            r = math.radians(ang)
            pts = [
                (cu + dx * math.cos(r) - dy * math.sin(r),
                 cv + dx * math.sin(r) + dy * math.cos(r))
                for dx, dy in rel
            ]
            # constrain on real Edge.Cuts material, not the bbox corners:
            # the lowest outline point sits ~31 mm higher along the plate
            # than the virtual (u = pcb_w, v = 0) corner
            vmin = min(
                (u - cu) * math.sin(r) + (v - cv) * math.cos(r)
                for u, v in self.outline
            )
            # the keyboard_drop shift lowers the pattern, so the same
            # material needs that much more lift to hold the floor; at
            # t30/-35 the floor binds and eats ~0.9 of the drop (net 29.1)
            lift = max(0.0, v_floor - (cv + vmin) + keyboard_drop)
            lifted.append((ang, lift, [(u, v + lift) for u, v in pts]))

        cands = [
            s * SHIFT_GRID
            for s in range(int(-SHIFT_SPAN / SHIFT_GRID), int(SHIFT_SPAN / SHIFT_GRID) + 1)
        ]
        best, best_key = None, None
        for combo in itertools.product(cands, repeat=len(angles)):
            posts = [
                (u + s - self.pcb_w / 2, v + bottom_ext - keyboard_drop)
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
            vmin = min(
                cv + (u - cu) * math.sin(r) + (v - cv) * math.cos(r)
                for u, v in self.outline
            )
            corner_z = (
                self.origin_z
                + (bottom_ext + vmin + lift - keyboard_drop) * self.ct
                + self.face_z * self.st
            )
            out.append(
                (ang,
                 [(u + s - self.pcb_w / 2, v + bottom_ext - keyboard_drop)
                  for u, v in pts])
            )
            meta.append((ang, s, lift, round(corner_z, 2)))
        return out, meta

    # -- rib / haunch auto-placement ------------------------------------------

    def _auto_ribs(self) -> list[float]:
        """Rib x positions: one rib per joint screw, just INBOARD of it —
        the edge screw is the pull-out/clamp anchor, so its rib backs the
        highest junction load, and outboard of the screw nothing is left
        inside the boot anyway. Hard limits: every post crossing the rib
        band keeps >= 12 mm sideways (a back-face screw bore or nut pocket
        must never be buried — same figure as the haunch cap), the merged
        key-bar grooves keep >= 1 mm, and the rib stays fully inside the
        boot with >= 1 mm margin. A rib with no valid slot is dropped and
        reported: fewer ribs is the normal fallback on a crowded layout,
        never a widened boot."""
        post_keep = 12.0  # back-face hardware clearance, cf. _cap_haunch
        zone = [x for x, y in self.all_posts if -3 <= y <= rib_h + 6]
        key_lo = self.band_mid - key_x - key_l / 2 - joint_clear
        key_hi = self.band_mid + key_x + key_l / 2 + joint_clear
        lo = self.boot_lo + 1 + rib_t / 2
        hi = self.boot_hi - 1 - rib_t / 2
        ribs: list[float] = []
        for x_s in self.joint_screw_xs:
            # the key bar splits the boot, so this screw's rib must stay on
            # the screw's side of it; start right inboard of the nut pocket
            inb = 1.0 if x_s < self.band_mid else -1.0
            if inb > 0:  # left screw: between boot edge and key bar
                s_lo, s_hi = lo, key_lo - 1 - rib_t / 2
            else:  # right screw: between key bar and boot edge
                s_lo, s_hi = key_hi + 1 + rib_t / 2, hi
            rib_off = nut_pocket_ac / 2 + rib_t / 2 + 1  # clear the pocket
            x = 2 * round((x_s + inb * rib_off) / 2)  # even mm
            x = min(max(x, s_lo), s_hi)
            while s_lo <= x <= s_hi and any(abs(x - px) < post_keep for px in zone):
                x += 2.0 * inb  # post hardware in the way: slide inboard
            if s_lo <= x <= s_hi:
                ribs.append(x)
            else:
                print(
                    f'note: no rib fits inboard of the joint screw at '
                    f'x={x_s:g} (pocket-post hardware vs key bar) — dropped'
                )
        # ribs must never poke outside the boot — the fit rules above
        # guarantee it; the assert keeps future keep-out edits honest
        for rx in ribs:
            assert (
                rx - rib_t / 2 >= self.boot_lo + 1
                and rx + rib_t / 2 <= self.boot_hi - 1
            ), (f'rib x={rx:g} pokes outside the boot '
                f'[{self.boot_lo:g}, {self.boot_hi:g}]')
        return sorted(set(ribs))

    def _cap_haunch(self) -> float:
        """Cap the haunch height so its ridge never runs behind a mounted
        post's back-face hardware: the screw clearance bore and, wider still, the
        nut pocket mouth (hex ~4.3 mm major radius plus wall, hence 6.5 mm
        clear of the post center). Floored at 2 mm: below that the wedge is
        a negligible sliver — it only softens the junction corner, the ribs
        carry the load — so once the pattern drops (keyboard_drop) the cap
        may shrink the haunch to nearly nothing instead of covering a
        pocket mouth. Only pocket posts count — the lowest post's
        self-tapping pilot is a plain bore and needs no back-face access."""
        min_y = min(y for _, y in self.pocket_posts)
        y_top = min_y - 6.5
        h = (self.origin_z + y_top * self.ct - base_t + 1) / self.ct
        return max(2.0, min(haunch_h, h))

    # -- keyboard-plate frame helpers -----------------------------------------

    def pad_positions(self) -> list[tuple[float, float]]:
        # per-side x: the slab IS the raw (asymmetric) post-need footprint,
        # so each side's pad row hugs ITS slab edge — unaffected by the band
        # clamp (the base does not change when the plate band is trimmed)
        px_lo = self.slab_lo + base_r + pad_edge
        px_hi = self.slab_hi - base_r - pad_edge
        fy = self.y0 + base_r + pad_edge
        ry = self.y1 - base_r - pad_edge
        return [(px_lo, fy), (px_hi, fy), (0, fy), (px_lo, ry), (px_hi, ry), (0, ry)]

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
        # hull bottom corners at the per-side band edges: below the taper
        # start the plate is the (asymmetric) bottom band; post keep-boxes
        # poking past a band edge widen the hull locally (the bulge)
        pts = [(self.band_lo, y0), (self.band_hi, y0)]
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
        # trim each side back to ITS band edge; on a side whose post keep-box
        # pokes past it, the tab grows out of the band edge through tangent
        # quarter-round shoulders (clear of the post keep circle)
        for sx in (-1, 1):
            edge = abs(self.band_edges[sx])
            over = self.plate_half - edge
            spans = dict(self.bulges).get(sx, [])
            x_mid = sx * (edge + over / 2)
            if not spans:
                if over > EPS:  # nothing past the band edge on this side
                    solid -= Pos(x_mid, self.plate_h / 2, -EPS) * Box(
                        over, self.plate_h, plate_t + 2 * EPS,
                        align=(Align.CENTER, Align.CENTER, MIN),
                    )
                continue
            lo0, hi0 = spans[0]
            x_in, x_out = edge, self.plate_half
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
        # Top-corner trim (user sketch, back view): two SINGLE straight
        # lines, one per side. Each line is the outward offset of the
        # segment top-post -> side-post by e = post_d/2 + 4.5 (= 9.5 mm:
        # the Ø15 flare foot radius 7.5 + 2 mm user clearance — the
        # boundary passes 9.5 mm from each post center; a first 7 mm
        # attempt shaved the flare's outer seat flat). From the top post
        # (H2) one line runs straight to the left post (H4), one to the
        # right post (H1) — no multi-segment cuts; the cap left between
        # the lines covers the top post's boss. Anchors derive from the post
        # layout, so they track layout changes; the post-clearance guard
        # below is the tripwire that fails loudly if a future layout ever
        # walks a nut pocket into a line.
        y0 = rib_h / self.ct + taper_margin
        if y0 < self.plate_h:  # only when the roof is actually tapered
            top = max(self.all_posts, key=lambda p: p[1])  # H2: top post
            sides = {
                'left': min(self.all_posts, key=lambda p: p[0]),   # H4
                'right': max(self.all_posts, key=lambda p: p[0]),  # H1
            }
            e = post_d / 2 + 4.5  # Ø15 flare foot radius 7.5 + 2 mm user
            # clearance; the first 7 mm offset shaved the flare's outer
            # seat flat (user-flagged)
            cuts: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {}
            normals: dict[str, tuple[float, float]] = {}
            for name, side in sides.items():
                dx, dy = side[0] - top[0], side[1] - top[1]
                ln = math.hypot(dx, dy)
                # outward unit normal: the perpendicular of top->side that
                # points AWAY from the boot centre (band_mid, y0)
                nx, ny = -dy / ln, dx / ln
                if nx * (side[0] - self.band_mid) + ny * (side[1] - y0) < 0:
                    nx, ny = -nx, -ny
                normals[name] = (nx, ny)
                cuts[name] = (
                    (top[0] + e * nx, top[1] + e * ny),
                    (side[0] + e * nx, side[1] + e * ny),
                )
            assert all(
                math.isfinite(v) for a, b in cuts.values() for v in a + b
            ), 'top-corner cut line anchors are not finite'
            # The two lines must keep a >= 8 mm gap where the top post's
            # boss tops out, so the boss stays covered (t30/-35: ~10.0 mm;
            # the lines close at y ~121.1 — above the boss top 116.7, just
            # under the taper hull top). Measured at the boss top, not
            # plate_h: with e = 7 on both sides the wedge can close before
            # the plate top on layouts whose side post hangs low. The old
            # keep-box anchor asserts are gone on purpose: the boundary now
            # INTENTIONALLY passes 2 mm outside the bosses, which is inside
            # the old keep-box margin — the 6.5 mm post guard below is the
            # only protection.
            boss_top_y = top[1] + post_d / 2
            cap_edges = {
                name: a[0] + (boss_top_y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
                for name, (a, b) in cuts.items()
            }
            cap_w = cap_edges['right'] - cap_edges['left']
            assert cap_w >= 8.0, (
                f'top-corner cuts leave only {cap_w:.2f} mm of cap at the '
                "top post's boss top — the boss would not stay covered"
            )
            listing = '; '.join(
                f'{name} ({a[0]:g},{a[1]:g})->({b[0]:g},{b[1]:g})'
                for name, (a, b) in cuts.items()
            )
            print(
                f'note: top-corner cuts 2 mm outside the post bosses: '
                f'{listing} | cap at boss top {cap_w:.1f} mm'
            )
            # GUARD: the cuts intentionally pass closer to the posts than
            # the taper's 10 mm keep, but a back-face nut pocket must never
            # be clipped: every post center keeps >= 6.5 mm to both lines
            # (pocket across-corner radius 4.27 + pinch lip + wall margin,
            # the figure the haunch cap uses). The top/side posts sit at
            # exactly e = 7.0 mm on their own side's line — that is the
            # design, not a violation. A layout change that violates this
            # must fail loudly, not silently shave a pocket.
            cut_guard = 6.5
            for name, (a, b) in cuts.items():
                ux, uy = b[0] - a[0], b[1] - a[1]
                ln = math.hypot(ux, uy)
                dists = [
                    abs(ux * (py - a[1]) - uy * (px - a[0])) / ln
                    for px, py in self.all_posts
                ]
                if min(dists) < cut_guard:
                    listing = ', '.join(
                        f'({px:g},{py:g})->{d:.2f}'
                        for (px, py), d in zip(self.all_posts, dists, strict=True)
                    )
                    sys.exit(
                        f'error: top-corner cut {name} passes within '
                        f'{min(dists):.2f} mm of a post center (guard '
                        f'{cut_guard} mm = nut pocket 4.27 + lip + margin); '
                        f'distances: {listing}'
                    )
            # through-thickness polygon extrusions, same EPS style as the
            # taper cutter; corners stay sharp (per sketch).
            # _half_plane_cutter removes everything LEFT of a->b: flip a
            # line whose outward normal points RIGHT of a->b, so each cut
            # always removes the OUTWARD side (left line -> upper-left,
            # right line -> upper-right, same semantics as ever).
            ext = 20.0
            x_lo, x_hi = -self.plate_half - ext, self.plate_half + ext
            for name, (a, b) in cuts.items():
                nx, ny = normals[name]
                ux, uy = b[0] - a[0], b[1] - a[1]
                if -uy * nx + ux * ny < 0:  # outward = right of a->b
                    a, b = b, a
                solid -= Pos(0, 0, -EPS) * extrude(
                    _half_plane_cutter(a, b, x_lo, x_hi, 0.0, self.plate_h + ext),
                    plate_t + 2 * EPS,
                )
        for hole in self.all_posts:
            x, y = hole
            post = Pos(x, y, plate_t) * Cylinder(
                post_d / 2, self.post_h, align=(Align.CENTER, Align.CENTER, MIN)
            )
            if hole != self.lowest_post:
                # 45 deg flare (Ø15 foot ring) where the standoff meets the
                # plate: adhesion-friendly transition, self-supporting. The
                # lowest post stays a plain cylinder: it is self-tapped
                # anyway, and at this drop its flare foot would overhang the
                # plate's bottom edge (center y ~ 5.6, foot to y ~ -1.9) and
                # get clipped into a half-trumpet lump by the y >= 0 clip
                # below; a plain Ø10 cylinder wall prints fine.
                post += Pos(x, y, plate_t) * Cone(
                    post_d / 2 + 2.5,
                    post_d / 2,
                    1.5,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
            solid += post
        # Flare clip: a deep pattern can park a post's Ø15 flare foot past
        # the plate's bottom edge (t30/-35 at drop 30: the lowest post
        # centers at y ~ 5.6, its flare would reach y ~ -1.9 — that post
        # gets no flare at all now, see above, so this only backstops other
        # layouts/tilts). The plate itself already spans y >= 0, so
        # intersect the finished solid with a generous y >= 0 box: any
        # overhanging flare is trimmed flush with the plate's bottom edge —
        # a straight vertical clip face, printable; the post top's bearing
        # area is untouched.
        solid &= Pos(-self.plate_half - 20, 0, -20) * Box(
            self.plate_w + 40,
            self.plate_h + 20,
            plate_t + self.post_h + 40,
            align=(Align.MIN, Align.MIN, Align.MIN),
        )
        top = self.face_z
        for hole in self.all_posts:
            x, y = hole
            if hole == self.lowest_post:
                # Lowest post: self-tapping pilot straight through post +
                # plate. A nut pocket here would sit inside the boot's
                # z-band (unreachable), and a tapped hole needs no back-face
                # access. The haunch toe now wraps the exit (back face
                # y ~ 2.82; the toe spans y 0.6..4.5 near the seat at every
                # x): the re-cut below keeps the bore open, and the M4 tip
                # (~y 4.03) threads ~1 mm into that toe plastic — extra
                # bite, by design.
                solid -= Pos(x, y, -EPS) * Cylinder(
                    tap_d / 2,
                    top + 2 * EPS,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
                continue
            if self.attach == 'nut':
                # Clamp from the back: the post top stays SOLID and bears the
                # PCB; the M4 nut lives in a hex pocket opened at the plate's
                # BACK face, radially inside the post footprint. The screw
                # (M4x12: grip PCB 1.6 + post 3 + wall 1.4 ≈ 6 mm + 3.4 nut)
                # comes through the board and pulls the nut against the
                # 1.4 mm front wall the post directly buttresses — load path
                # nut -> wall -> post, pure compression, no printed bridge.
                # The mouth is a hex pinch (nut_mouth_af < nut across flats,
                # aligned with the pocket hex) so the nut snaps in from the
                # back and stays with the upright. The screw passes through
                # the pocket volume; both are cuts, so that is fine.
                hex_r = (nut_af / 2) / math.cos(math.radians(30))
                mouth_r = (nut_mouth_af / 2) / math.cos(math.radians(30))
                solid -= Pos(x, y, -EPS) * (
                    extrude(
                        RegularPolygon(mouth_r, 6, major_radius=True),
                        nut_mouth_h + EPS,
                    )
                    + Pos(0, 0, nut_mouth_h)
                    * extrude(
                        RegularPolygon(hex_r, 6, major_radius=True),
                        nut_pocket_depth - nut_mouth_h,
                    )
                )
                # screw clearance: back face through plate + post, out the top
                solid -= Pos(x, y, -EPS) * Cylinder(
                    screw_d / 2,
                    top + 2 * EPS,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
            else:
                # self-tapping pilot: back face through the solid post top
                solid -= Pos(x, y, -EPS) * Cylinder(
                    tap_d / 2,
                    top + 2 * EPS,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
        return solid

    # -- base, world frame (desk = XY plane, user at -Y) ----------------------

    def base_solid(self):
        # the slab matches the raw post-need footprint (asymmetric — the Box
        # centers on the slab's mid-x, not on 0); the plate's bottom band is
        # clamped to the boot, but the slab itself keeps the full footprint
        base = Pos(
            (self.slab_lo + self.slab_hi) / 2, (self.y0 + self.y1) / 2, base_t / 2
        ) * Box(self.bx, self.y1 - self.y0, base_t)
        base = fillet(base.edges().filter_by(Axis.Z), base_r)
        base = fillet(base.edges().group_by(Axis.Z)[-1], base_top_fillet)
        for px, py in self.pad_positions():
            # Pad pocket, two levels: a flat glue annulus (width pad_ring_w)
            # at pad_recess inside the straight capture wall at
            # pad_d + 2*pad_fit, plus a centre relief pad_relief deeper
            # (vertical step — a narrowing cone would add a 45° overhang
            # per layer where a hole wall is free). A former relief slot +
            # glue lands kept the ceiling from sagging, but the lands stood
            # 0.4 mm proud of the floor and kept the pad from seating —
            # here the glue ring IS the floor, flush everywhere. Droop or
            # detached strands from the recess ceiling fall through the
            # relief and out of the pocket mouth instead of onto the glue
            # face; the ring itself is a wall-anchored cantilever whose
            # droop bends into the relief, away from the pad.
            ro = pad_d / 2 + pad_fit
            pocket = Pos(px, py, -EPS) * Cylinder(
                ro, pad_recess + EPS, align=(Align.CENTER, Align.CENTER, MIN)
            )
            base -= pocket
            relief = Pos(px, py, -EPS) * Cylinder(
                ro - pad_ring_w,
                pad_recess + pad_relief + EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
            base -= relief
        base += joint_keys(self.band_mid)
        base -= joint_screw_bores(self.joint_screw_xs)
        base -= joint_screw_head_pockets(self.joint_screw_xs)
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
            # Applied before the boot union: the cut spans world y <= 0,
            # entirely in front of the boot footprint (front +4.5), so the
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
        # boot + haunch + ribs span the boot footprint: the slab inset by
        # boot_inset per side, so the foot sits fully on the slab's flat top
        # inside the base_top_fillet round-over (the plate band is clamped
        # flush to it). Outboard wall at the joint nut seats is a uniform
        # 1.5 mm — the screws hug the boot edges by construction (see
        # __init__).
        boot_w = self.boot_hi - self.boot_lo
        boot = Pos(
            (self.boot_lo + self.boot_hi) / 2,
            (boot_front + self.boot_rear) / 2,
            base_t + boot_h / 2,
        ) * Box(boot_w, self.boot_rear - boot_front, boot_h)
        # Boot-wide haunch: a shallow ramp up the back face that tops out
        # just above the boot (haunch_h), filling the junction inner
        # corner. y anchor 0 = the backplate's back face; the wedge
        # polygon supplies the 1.5 mm embed (see _wedge) — it must NOT
        # track boot_front, or a recessed front leaves it floating clear
        # of the plate.
        boot += Pos(self.boot_lo, 0, base_t) * extrude(
            Plane.YZ * self._wedge(haunch_d, self.haunch_h, rad), boot_w
        )
        # Ribs continue up the back face into the hammering zone, same
        # back-face anchor. Wedge base depth rib_t + 8 (slimmed from +18
        # after the first print): the base rear corner reaches
        # y = rib_t + 8 = 20 from the seat origin, and the derived boot
        # rear keeps a 6 mm wall behind it.
        for rx in self.rib_x:
            boot += Pos(rx - rib_t / 2, 0, base_t) * extrude(
                Plane.YZ * self._wedge(rib_t + 8, rib_h, rad), rib_t
            )
        # keyboard_drop can park the lowest POCKET post's back-face nut
        # pocket inside the boot's z-band (t30/-35 drop 20: mouth spans
        # world z 8.4..15.3 vs boot top 14), burying its pinch mouth. Cut
        # an access window through the boot in front of that pocket: the
        # nut slides in with tweezers, and the pinch lip — which lives in
        # the plate — still retains it. Boot-only cut, applied before the
        # union, so the plate's pocket and nut seat stay intact. With
        # lowest_post_tapped = True the lowest post has a plain self-tap
        # pilot (no pocket, no access needed), so this auto-skips whenever
        # the lowest POCKET clears the boot top on its own (drop 30 /
        # t30/-35: lowest pocket mouth z ~ 47 mm).
        lo_x, lo_y = min(self.pocket_posts, key=lambda p: p[1])
        if (lo_y - 3.98) * self.ct < boot_h + 0.5:
            # rear edge = mouth rear (lo_y*st + 4*st) plus nut body + margin
            win_y1 = (lo_y + 4.0) * self.st + 2.0
            boot -= Pos(lo_x - 5.5, boot_front, base_t - EPS) * Box(
                11.0, win_y1 - boot_front, boot_h + 2 * EPS,
                align=(Align.MIN, Align.MIN, MIN),
            )
        body += boot
        if self.flange_w > 0:
            body += self._breakaway_flange()
        body -= joint_grooves(self.band_mid)
        body -= joint_nut_pockets(self.joint_screw_xs)
        # Re-cut the lowest post's self-tap pilot through whatever the boot
        # union added around its rear exit — by design since the haunch toe
        # now wraps the exit (thread bite in that plastic), and as a guard
        # for other configs (deeper drop, lower tilt) whose boot would
        # otherwise refill the bore.
        x, y = self.lowest_post
        body -= (
            Pos(0, 0, self.origin_z)
            * Rot(X=90 - self.tilt)
            * Pos(x, y, -EPS)
            * Cylinder(
                tap_d / 2,
                self.face_z + 2 * EPS,
                align=(Align.CENTER, Align.CENTER, MIN),
            )
        )
        # Trims the bottom post flares, which dip below the seat plane at
        # low tilt angles (nothing else lives down there).
        body -= Pos(0, 0, base_t - 25) * Box(400, 400, 50)
        return body

    def _wedge(self, depth: float, height: float, rad: float):
        """Reinforcement wedge profile in the YZ plane, anchored to the
        backplate's BACK FACE, not the boot front (anchoring at the recessed
        boot front (+4.5) had dragged rib and haunch ~4.5 mm rearward: they
        connected only via the boot and reinforced nothing). The caller
        positions at Pos(x, 0, base_t), so local (0, 0) is world
        (Y = 0, Z = base_t) and the tilted back face is the line
        Y = tan(tilt) * Z. Triangle, with e the perpendicular embed:
            P1 = (-e / cos, 0)             foot of the embed leg
            P2 = (height * sin - e * cos,
                  height * cos + e * sin)  apex up the back face
            P3 = (depth, 0)                base rear corner, on the seat
        P1-P2 runs parallel to the back face, e = 1.5 mm perpendicular
        inside it: full-height contact with the plate regardless of
        boot_front. P1 is slid e / cos along the seat plane so the base
        edge lies flat ON it — over the boot aft of y = 4.5, and on the
        bed across the ~1.7 mm strip in front of the 45 deg lip chamfer
        (first-layer material; the face above leans back at tilt from
        vertical, self-supporting). Putting P1 at the perpendicular foot
        instead (e * sin up) would float that strip's base edge over the
        recessed boot-front pocket. The hypotenuse faces up-forward
        throughout (rib: leans back ~12.6 deg from vertical; haunch:
        ~49 deg ramp), so the wedge prints support-free; the rib's
        protrusion past the back face tapers from ~17 mm at the boot top
        (z = 14) to 0 at z ~ 62 — the top few mm of the wedge submerge
        into the plate, intended.
        """
        e = 1.5  # perpendicular embed into the backplate
        s, c = math.sin(rad), math.cos(rad)
        return Polygon(
            (-e / c, 0),
            (depth, 0),
            (height * s - e * c, height * c + e * s),
            align=None,
        )

    def _breakaway_flange(self):
        """Thin ring around the boot footprint (--flange): bed-adhesion
        insurance for slick plates. flange_t tall so it snaps off with
        pliers after printing; added after the lip chamfer cut, so the two
        never interact, and clear of every joint feature. Follows the boot
        footprint (the band inset by boot_inset), so it centers on the
        band's mid-x too."""
        boot_w = self.boot_hi - self.boot_lo
        outer = RectangleRounded(
            boot_w + 2 * self.flange_w,
            self.boot_rear - boot_front + 2 * self.flange_w,
            4,
        )
        inner = Rectangle(boot_w, self.boot_rear - boot_front)
        return Pos(
            (self.boot_lo + self.boot_hi) / 2,
            (boot_front + self.boot_rear) / 2,
            base_t,
        ) * extrude(outer - inner, flange_t)

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
        help="'base' = base slab (follows the band footprint) | "
        "'upright' = per-tilt part | "
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
        '--post-h', type=float, default=3, help='post height = PCB-to-plate gap'
    )
    ap.add_argument(
        '--attach',
        choices=['nut', 'tap'],
        default='nut',
        help='M4 nut snapped into a back-face pocket (nut) or self-tapping '
        'pilot through the post (tap)',
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

    pcb_w, pcb_h, holes, outline = parse_pcb(args.pcb)
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
        # the coupons reuse the exact production features, so they need the
        # derived layout (band mid / screw positions) from a real Stand
        stand = Stand(pcb_w, pcb_h, holes, outline, args.post_h, args.attach,
                      set_angles)
        emit(joint_coupon_base(stand), 'joint_coupon_base')
        emit(joint_coupon_boot(stand), 'joint_coupon_boot')
        return

    # The base is tilt- and side-independent: one Stand, one export.
    if args.part == 'base':
        stands = [
            Stand(pcb_w, pcb_h, holes, outline, args.post_h, args.attach, set_angles)
        ]
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
                    outline,
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
                f'{max(x for x, _ in pts):7.2f}]  '
                f'lowest PCB outline point z={corner_z:6.2f} mm'
            )
        key_xs = [round(stand.band_mid + s * key_x, 1) for s in (-1, 1)]
        print(
            f'ribs at x = {[round(x, 1) for x in stand.rib_x]}'
            f'  | haunch_h = {stand.haunch_h:.1f} mm'
            f'  | joint: keys x = {key_xs}, '
            f'screws x = {[round(x, 1) for x in stand.joint_screw_xs]} '
            f'({joint_screw} from below)'
        )
        print(
            f'boot: x [{stand.boot_lo:g}, {stand.boot_hi:g}], '
            f'front {boot_front:g}, rear {stand.boot_rear:g} mm'
            f'  | CoM y {stand.com_y:.1f} mm, rear margin '
            f'{stand.boot_rear - stand.com_y:.1f} mm'
        )
        emit(stand.build(args.part, args.side), stem)


if __name__ == '__main__':
    main()
