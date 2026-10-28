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
    needs none). Its Ø15 flare dips past the plate's bottom edge and now
    prints on the boot's coplanar front face (the buried lip), so all four
    posts carry the same flare. The haunch toe now wraps the pilot's
    rear exit, giving the screw tip ~1 mm of extra plastic bite.
  * Single mounting set, minimal upright: one 4-post pattern (--set-angles,
    default -53 deg in-plane rotation = home-row slope), and the backplate
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

Handedness: the unmirrored layout happens to be the RIGHT-hand upright,
picked at random, no deeper reason. --side left mirrors the upright; the
BASE is hand-universal: every engagement feature (key bar, joint screws,
boot flat) is symmetric about the band mid, so the mirrored upright
re-seats on the very same slab shifted 2*band_mid sideways (the 'full'
preview shows exactly that mounting).

Rotation set — one 4-hole pattern per upright, rotated in-plane about its
centroid by --set-angles (default -53 deg CCW in the front view; negate to
mirror the tilt direction). The solver centers the set laterally and lifts
it so the rotated board's lowest outline point (real Edge.Cuts material —
the bbox corner holds no board) clears the base top by SLIDE_TOP_MARGIN
(1.5 mm along the plate plane), then shifts the whole pattern
keyboard_drop down the plate (user-measured).

Usage:
    uv sync                                  # once; creates .venv + uv.lock
    uv run stand.py --part base              # base slab, serves BOTH hands
    uv run stand.py --part base --pad-pockets  # ... with 6 pad pockets
    uv run stand.py --side right --part upright  # right upright (unmirrored)
    uv run stand.py --part upright           # left upright = mirrored right
    uv run stand.py --part upright --tilt 10,15,20,25,30   # a batch of angles
    uv run stand.py --part full              # fused assembly preview
    uv run stand.py --part joint             # small fit-test coupons
    uv run stand.py --part padtest           # slicer-support test coupons
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
keyboard_drop = 35.5  # whole mounting pattern shifted DOWN the plate by this
# much (user-measured 30, then 33 at t30/-45; re-raised to 35.5 for t30/-53
# so the slide-margin floor BINDS again and the keyboard sits as low as the
# 1.5 mm slide margin allows: the solver lifts the pattern just enough to
# hold the floor, so the NET drop auto-caps —
# t30/-53: lift ~0.00, net ~35.5, lowest point z ~ 7.3 (slide margin 1.5).
# The old 20 cap guarded the lowest post's back-face nut pocket vs the boot
# top; lowest_post_tapped = True removes it)
lowest_post_tapped = True  # lowest post gets a Ø tap_d self-tapping pilot
# instead of a nut pocket: after the drop its back-face pocket would sit
# inside the boot's z-band with no insertion access, and a tapped hole needs
# none. Its Ø15 flare dips past the plate edge but prints on the boot's
# coplanar front face (flare_supported)
top_rim = 5  # vertical (world-z) plate rim above the topmost post top: the
#              backplate spans only the screw holes' desk projection (user
#              sizing rule); PCB above the top posts cantilevers on them
plate_r = 8
post_d = 10  # solid standoff boss; the Ø4.5 screw bore leaves a 2.75 mm wall
post_edge_margin = 5  # plate edge beyond a post center: keep = post_d/2 +
#                      post_edge_margin = 10 still clears the Ø8.6 nut
#                      pocket keep-out on the back face; also the keep-out
#                      used by the taper outline (_taper_outline) and the
#                      per-side bottom-band derivation (__init__)
taper_margin = 2  # the full-width band extends this far above the rib tops
#                  before the outline tapers in to the post columns
nut_af = 7.2  # hex pocket across flats, NO pinch lip anywhere (user call,
#              replacing 7.4 AF + 6.9 lip): the 7.0 nut drops in freely and
#              sits ~0.1 mm/side centred — the lip's 0.2 mm/side float made
#              screw start fiddly. Retention is friction only until the
#              screw clamps the nut, so nuts are inserted at assembly time
nut_h = 3.4
nut_pocket_depth = nut_h  # PCB nut pocket depth into the plate; leaves
#                          plate_t - depth = 2.6 mm of front wall that the
#                          Ø10 post directly buttresses — the nut bears on
#                          that wall (clamp load: nut -> wall -> post)
screw_d = 4.5
tap_d = 4.0  # Ø self-tapping pilot for the lowest post (M4 screws
#             straight in; user-sized up 3.4 -> 3.8 -> 4.0: PETG shrink
#             plus vertical-print sag made 3.8 too tight to start)
base_t = 6  # the seat plane: base top = upright's boot bottom. 8 was sized
#            for walls around the (since deleted) coin slots; 6 is plenty
#            for a flat slab and drops the keyboard 2 mm
base_front = 25  # apron towards the user, trimmed for cost: typing loads
#                 lean on the rear lever arm, the front pads carry little
#                 (forward tip lever 45 mm covers occasional pull/bump only)
base_rear = 85  # rearward depth on the tilt side: the tip-over lever arm.
#                Sized against the PRE-drop layout (base_rear 100 was picked
#                before keyboard_drop existed): the keyboard sitting ~29 mm
#                lower cut the worst-case lever h ~170 -> ~144 mm, i.e. +20%
#                tip margin at the old footprint — 85 keeps the old margin
#                (worst-case horizontal push at the keycap top, ~150 g board:
#                1.35 vs 1.40 N). Floor: rear pads reach y1-22; below y1 ~ 56
#                they slide under the boot footprint.
base_r = 10  # base slab corner radius (plan view)
# Junction reinforcement — vertical typing hammers the base-upright junction;
# the junction itself is now a demountable joint (constants below).
haunch_d = 12  # full-width haunch: depth along the seat plane (was 18;
#               slimmed after the first print — 4-perimeter walls suffice)
haunch_h = 10  # haunch height above the seat plane: a shallow ramp that
#               tops out just above the boot, keeping the boot ledge around
#               the nut pockets open (the taller reinforcement is the ribs')
rib_t = 14  # rib thickness (X); rib x-positions are solved automatically to
#            clear every post AND the joint hardware; widened 8 -> 12
#            (30/-35 layout) then -> 14 for ~+15% root section at +2 cm3 —
#            the last even step that fits: at 16 the left stub slides to
#            x=-30 and its flank crosses the key-bar keep-out (drops)
rib_h = 70  # rib height up the back face (was 60)
rib_foot = rib_t + 12  # wedge root depth along the seat (was rib_t + 8 =
#            20): depth buys ~3x more root section than thickness at the
#            max-moment foot; rear corner y = 26 still leaves an 8 mm wall
#            to the derived boot rear (min 6) and the slope stays < 30 deg
#            from vertical, self-supporting
rib_top_flat = 2.5  # rib apex truncated to this width: the zero-thickness
#            tip printed fragile and carries no load up there anyway
# Pad pockets are back, as a slicer-support experiment: the
# Ø20 two-level no-support pockets kept failing in print (recess-ceiling
# droop / strands fouling the glue face), so the retry is a plain
# straight-wall Ø20.2 × 0.6 pocket with NO steps — the recess ceiling rides
# on slicer support, expected to also clean up the Ø7.6 head-pocket shoulder.
# Print --part padtest first to dial in the support settings. The 1 mm
# self-adhesive TPU pads glue into the flat-bottom pocket; their faces sit
# pad_t - pad_recess = 0.4 above the desk, so the pads stay the ONLY desk
# contact (the tip-over pivots) across die-cut thickness tolerance, and
# every mm of outward placement is a mm of tip lever.
pad_d = 20
pad_fit = 0.1  # radial install clearance at the pocket mouth (0.2 on dia;
#                separate from the joint_clear key fit below)
pad_t = 1.0  # purchased self-adhesive TPU pad thickness
pad_protrude = 0.4  # pad face above the desk face
pad_recess = pad_t - pad_protrude  # 0.6 pocket depth, plain flat bottom
pad_edge = 2  # pad circle edge to the slab straight edge; pads sit as far out
#              as this allows — the tip-over pivots are the PAD contact lines,
#              so every mm outward is a mm of tip lever (2 mm still keeps the
#              0.6 mm pocket fully on the flat, clear of the base corner
#              fillets)
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
#             current working default: 30 deg with SET_ANGLES [-53]
joint_clear = 0.2  # key/groove clearance per face: snug FDM slide fit
#                    (halved from the old 0.4 baseline — less slop on the
#                    merged key bar; print a --part joint coupon FIRST to
#                    verify the tighter fit before ordering full parts)
key_l = 30  # shear key footprint: key_l along X, key_w along Y, key_h tall
key_w = 8
key_h = 8  # full boot-height key, top flush with the boot top ledge: longer
#           shear guidance, less rocking; the grooves are cut-through windows,
#           so the key can never bottom out and the boot still seats on the
#           seat plane, never on the keys (verified: key/boot interference 0)
key_x = 12  # key centerline offset from the BAND MID (band_mid ± key_x —
#            the band is asymmetric, so mid-boot anchoring keeps the merged
#            key bar centered); keys span y ∈ (key_y0, key_y1)
key_y0, key_y1 = 12, 20
key_chamfer = 0.5  # 45° lead-in on the key top edges
boot_h = 8  # boot foot height; its top ledge hosts the nut pockets. 8 puts
#            4.4 mm of wall over the pockets and stiffens the junction band
# boot_front is DERIVED per instance (see __init__): the plate's front
# surface plane continued down to the seat (-plate_t / cos(tilt)), so the
# boot's front face IS the plate surface flush to the base — one
# uninterrupted inclined face from the toe up through the keyboard area,
# the raw bottom edge buried inside, no step at boot-top height, and a
# constant post_h gap to the PCB. The old flush-with-back-face front (0)
# needed a 45 deg lip chamfer to keep the bottom edge face printable —
# that notch is gone. Only the boot prism (and the flange/access window
# tracking it) uses this value — the reinforcement wedges anchor to the
# back face instead (see _wedge), and the lowest post's pilot exit stays
# buried in boot/toe plastic, re-cut through it (extra bite for the
# self-tapper)
boot_rear = 34  # boot rear edge DEFAULT (floor): each Stand derives its real
#                 rear in __init__ from three floors — wall behind the nut
#                 pockets, 6 mm behind the (slimmed) rib wedge feet, and the
#                 free-standing CoM margin — and extends rearward past this
#                 default whenever the measured CoM demands it
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
head_pocket_d = 7.6  # socket-head pocket in the base underside: the heads
#                     must sit below the desk face or the base would rock.
#                     8.0 printed a visible slop ring around the user's
#                     7.1 mm head; 7.6 halves it while still absorbing FDM
#                     shrink + bore eccentricity when tightening
head_pocket_h = 4.2
boot_nut_clear = 3.6  # boot material between its bottom and the nut floor:
#                      this pocket floor takes the clamp load — keep it
#                      chunky, plastic creeps under a sustained bolt load
nut_pocket_ac = 8.6  # hex pocket across corners + clearance (rib keep-out);
#                     generous vs the 8.31 actual at 7.2 AF on purpose —
#                     keep-outs outlive pocket dimension tweaks

# --- rotation set ---------------------------------------------------------------
# A single 4-post set per upright; its in-plane rotation (--set-angles,
# default -53 deg CCW in the front view, negate to mirror) sets the home-row
# slope. The solver centers the set laterally and lifts it so the rotated
# board's lowest outline point clears the base top by SLIDE_TOP_MARGIN
# (measured along the plate plane), then
# shifts the whole pattern keyboard_drop down the plate.
SET_ANGLES = [-53]  # deg, CCW in front view (negate to mirror tilt)
MIN_POST_CENTER = 11.0  # post diameter 10 + 1 mm printing clearance
SHIFT_GRID = 3.0  # lateral shift candidates per set, packed by brute force
SHIFT_SPAN = 45.0  # max |lateral shift| of a set centroid
PLATE_HALF_BOUND = 72.0  # lateral layout sanity cap: the solver keeps post
#                          |x| <= this - 8, and the bottom band (hence the
#                          base slab — the band IS the slab now) never
#                          exceeds it either; a wider wish is a layout
#                          problem, not a wider slab
SLIDE_TOP_MARGIN = 1.5  # keyboard slide margin vs base top, mm measured
#                        ALONG the plate plane (world-z margin = this *
#                        cos(tilt)): how far the mounted board could slide
#                        down its posts before its lowest outline point
#                        touches the base top — the ONLY height constraint
#                        (the old world-z 2.0 rule applied to a vertical
#                        drop the mounted board cannot make)
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


def joint_screw_bores(xs: list[float], y: float = joint_screw_y):
    """Vertical M4 clearance holes through the base (pair with
    joint_screw_head_pockets on the underside)."""
    bores = None
    for x in xs:
        bore = Pos(x, y, -EPS) * Cylinder(
            joint_screw_d / 2,
            base_t + 2 * EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        bores = bore if bores is None else bores + bore
    return bores


def joint_screw_head_pockets(xs: list[float], y: float = joint_screw_y):
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
        pocket = Pos(x, y, -EPS) * Cylinder(
            head_pocket_d / 2,
            head_pocket_h + EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def joint_nut_pockets(xs: list[float]):
    """Boot-side nut seats, one negative per screw: clearance from the boot
    bottom up to the nut floor, then one open-topped hex pocket straight
    through to the boot ledge — no pinch lip (deleted, see nut_af): the
    nut drops in from the top and seats on the bearing floor. Until the
    screw clamps it the nut is friction-held only, so drive the joint
    screws with the stand lying on its back."""
    hex_r = (nut_af / 2) / math.cos(math.radians(30))
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
            * extrude(
                RegularPolygon(hex_r, 6, major_radius=True),
                boot_h - floor + EPS,
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


def pad_pockets(positions: list[tuple[float, float]]):
    """Straight-wall Ø(pad_d + 2*pad_fit) x pad_recess pockets open at the
    base underside, flat bottoms, NO steps: the recess ceiling rides on
    slicer support (the old two-level no-support pocket kept failing — see
    the pad_* notes). Shared by the base and the pad coupon so the coupon
    tests the exact production feature."""
    pockets = None
    for px, py in positions:
        pocket = Pos(px, py, -EPS) * Cylinder(
            pad_d / 2 + pad_fit,
            pad_recess + EPS,
            align=(Align.CENTER, Align.CENTER, MIN),
        )
        pockets = pocket if pockets is None else pockets + pocket
    return pockets


def pad_coupon():
    """Slicer-support test coupon for the pad pocket: production-geometry
    pocket in a minimal round puck (2 mm wall, like the base edge margin),
    printed in the base's attitude (pocket open to the bed). Only 2 mm
    thick — the ceiling sits just 0.6 mm up, and material above it adds
    nothing to the support question (unlike the head pocket, whose 4.2 mm
    pocket needs the full base_t)."""
    c = Cylinder(pad_d / 2 + pad_fit + 2, 2,
                 align=(Align.CENTER, Align.CENTER, MIN))
    return c - pad_pockets([(0, 0)])


def head_coupon():
    """Slicer-support test coupon for the head-pocket shoulder: production-
    geometry Ø7.6 pocket + Ø4.5 through bore in a minimal round puck
    (2 mm wall), same attitude as the base underside."""
    c = Pos(0, 0, base_t / 2) * Cylinder(head_pocket_d / 2 + 2, base_t)
    return c - joint_screw_head_pockets([0], y=0) - joint_screw_bores([0], y=0)


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
    """Fit-test coupon, boot side: groove + nut seat (floor, open hex
    pocket), printed flat on what would be the seat plane. Checks the
    groove fit and that the 7.0 nut drops in and starts cleanly."""
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
        *,
        pad_pockets: bool = False,
    ):
        self.pcb_w, self.pcb_h, self.holes = pcb_w, pcb_h, holes
        self.outline = outline
        self.tilt = tilt
        self.post_h = post_h
        self.face_z = plate_t + post_h  # PCB seating plane over the back face
        self.attach = attach
        self.flange_w = flange_w
        self.pad_pockets = pad_pockets  # cut the 6 pad pockets
        # (off by default: plain flat underside)

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
        # Boot front edge (see the boot_front note by the constants): the
        # plate's front surface plane continued down to the seat. -plate_t/ct
        # is where that plane crosses the seat plane; from there the boot's
        # front face rises PARALLEL to the plate, burying the raw bottom edge
        # flush, leaving no step at boot-top height, and keeping a constant
        # post_h gap to the PCB underside plane. Tilts toward vertical push
        # the toe further forward (t10: -6.09); t30: -6.93.
        self.boot_front = -plate_t / self.ct
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
        if len(self.rib_x) + len(self.stub_ribs) < 2:
            print(f'warning: only {len(self.rib_x) + len(self.stub_ribs)} '
                  f'ribs (incl. stubs) fit inside the boot')
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
                rib_foot + 6,  # rib foot rear y = rib_foot, + 6 mm wall
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
        that lift, ~35.5 mm at the default drop 35.5).
        Optimizes for the largest minimum post distance, tie-broken by
        a compact span and a centered shift."""
        cu = sum(u for u, _ in self.holes) / len(self.holes)
        cv = sum(v for _, v in self.holes) / len(self.holes)
        rel = [(u - cu, v - cv) for u, v in self.holes]
        # v' the lowest board material may reach: the plate back face under
        # it must stay SLIDE_TOP_MARGIN above the base top, measured ALONG
        # the plate plane (slide direction the mounted board can actually
        # move in) — hence the *ct: world-z margin = slide * cos(tilt).
        # face_z = plate_t + post_h: the PCB back rides that far in front of
        # the plate back face, so its world height gains face_z * st
        v_floor = (
            base_t + SLIDE_TOP_MARGIN * self.ct - self.origin_z
            - self.face_z * self.st
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
        self.stub_ribs: list[tuple[float, float]] = []  # (x, capped height)
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
                continue
            # Full-height rib doesn't fit (pocket-post hardware vs key bar):
            # fall back to a SHORT stub at the initial inboard position
            # (user request). The wedge tops out below any pocket mouth
            # overlapping its x-span, so the stub backs the junction without
            # blocking nut insertion; it need not mirror the far rib.
            x_stub = 2 * round((x_s + inb * rib_off) / 2)
            if not (
                x_stub - rib_t / 2 >= self.boot_lo + 1
                and x_stub + rib_t / 2 <= self.boot_hi - 1
                and (x_s >= self.band_mid or x_stub + rib_t / 2 <= key_lo - 1)
                and (x_s <= self.band_mid or x_stub - rib_t / 2 >= key_hi + 1)
            ):
                print(
                    f'note: no rib fits inboard of the joint screw at '
                    f'x={x_s:g} (pocket-post hardware vs key bar) — dropped'
                )
                continue
            # cap: wedge apex stays 2 mm below any pocket mouth overlapping
            # the stub's x-span (mouth bottom z = ct·(y − ac/2) on the back
            # face; wedge apex z = ct·h + e·st above the seat). Posts at or
            # below the boot top (back-face height 8/ct above the seat) are
            # skipped: the wedge is submerged inside the boot there, so it
            # cannot block their hardware (t30/−53: H3 y 6.8 sits inside the
            # boot band — treating it like a pocket post produced a bogus
            # −0.7 cap that killed this rib; the tapped lowest post has no
            # pocket and needs no rear access anyway).
            h = rib_h
            for px, py in self.all_posts:
                if py <= 8.0 / self.ct:
                    continue
                if abs(px - x_stub) < rib_t / 2 + nut_pocket_ac / 2:
                    h = min(
                        h,
                        (py - nut_pocket_ac / 2)
                        - (2.0 + 1.5 * self.st) / self.ct,
                    )
            if h < 25:
                print(
                    f'note: no room for a stub rib at x={x_stub:g} either '
                    f'(capped height {h:.1f} < 25) — dropped'
                )
                continue
            h = 2 * round(h / 2)
            self.stub_ribs.append((x_stub, h))
            print(
                f'note: short rib at x={x_stub:g}, h={h:g} backs the joint '
                f'screw at x={x_s:g} (full-height rib blocked by '
                f'pocket-post hardware)'
            )
        # ribs must never poke outside the boot — the fit rules above
        # guarantee it; the assert keeps future keep-out edits honest
        for rx in ribs + [x for x, _ in self.stub_ribs]:
            assert (
                rx - rib_t / 2 >= self.boot_lo + 1
                and rx + rib_t / 2 <= self.boot_hi - 1
            ), (f'rib x={rx:g} pokes outside the boot '
                f'[{self.boot_lo:g}, {self.boot_hi:g}]')
        return sorted(set(ribs))

    def flare_supported(self, hole: tuple[float, float]) -> bool:
        """A Ø15 flare foot prints only where the substrate under it is
        continuous: the plate front face — or, for a ring dipping past the
        plate's bottom edge, the boot's front face, which is coplanar with
        it (the buried lip) and x-extruded across the boot span. Outside
        that span the dipping ring would print over air, so the post falls
        back to a plain Ø10 wall."""
        x, y = hole
        flare_r = post_d / 2 + 2.5
        if y >= flare_r:
            return True  # ring fully on the plate
        w = math.sqrt(flare_r**2 - y**2)
        return self.boot_lo <= x - w and x + w <= self.boot_hi

    def _cap_haunch(self) -> float:
        """Cap the haunch height so its ridge never runs behind a mounted
        post's back-face hardware: the screw clearance bore and, wider still, the
        nut pocket opening (hex ~4.2 mm major radius plus wall, hence 6.5 mm
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
        """Six pads: the 4 corners plus one mid front/rear each (the old
        6-pad layout). Per-side x: the slab IS the raw (asymmetric)
        post-need footprint, so the outer pads hug THEIR slab edge —
        unaffected by the band clamp (the base does not change when the
        plate band is trimmed); the mid pads sit on the slab mid (= the
        band mid, the base is asymmetric)."""
        px_lo = self.slab_lo + base_r + pad_edge
        px_hi = self.slab_hi - base_r - pad_edge
        mx = (self.slab_lo + self.slab_hi) / 2
        fy = self.y0 + base_r + pad_edge
        ry = self.y1 - base_r - pad_edge
        return [
            (px_lo, fy), (px_hi, fy), (mx, fy),
            (px_lo, ry), (px_hi, ry), (mx, ry),
        ]

    # -- backplate + posts, plate-local frame (x right, y up, z out front) ----

    def _top_corner_cuts(self):
        """The two top-corner trim lines (see plate_and_posts for the full
        rationale), or None when the roof is not tapered. Pure layout math,
        shared with _taper_outline — whose hull must never swing inboard of
        these lines (see the anchor note there)."""
        y0 = rib_h / self.ct + taper_margin
        if y0 >= self.plate_h:
            return None
        top = max(self.all_posts, key=lambda p: p[1])  # H2: top post
        sides = {
            'left': min(self.all_posts, key=lambda p: p[0]),   # H4
            'right': max(self.all_posts, key=lambda p: p[0]),  # H1
        }
        e = post_d / 2 + 4.5  # Ø15 flare foot radius 7.5 + 2 mm user clearance
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
        return y0, cuts, normals

    def _taper_outline(self) -> list[tuple[float, float]]:
        """Retained outline above the full-width band: the convex hull of the
        posts' keep-boxes (center +/- post_d/2 + post_edge_margin, clamped to
        the taper start / plate top) plus the band's corners at the taper
        start. The plate never touches the PCB, so up there it only carries
        the post columns and the web between them — the hull is exactly that.
        Returns [] when the band already reaches the plate top."""
        y0 = rib_h / self.ct + taper_margin
        if y0 >= self.plate_h:
            return []
        keep = post_d / 2 + post_edge_margin
        # hull bottom corners at the per-side band edges: below the taper
        # start the plate is the (asymmetric) bottom band; post keep-boxes
        # poking past a band edge widen the hull locally (the bulge). A post
        # fully below the taper start keeps its x-span as a segment ON y0 —
        # dropping it instead (the old rule) let the hull's side edge chord
        # straight to the band corner, crossing INSIDE the top-corner cut
        # line when the side post hangs low (t30/-40: H1 tops out at 80.4
        # < y0 82.8) and the outline stepped line -> chord -> line just
        # above that post (user-flagged render).
        pts = [(self.band_lo, y0), (self.band_hi, y0)]
        for x, y in self.all_posts:
            lo = max(y - keep, y0)
            hi = max(min(y + keep, self.plate_h), lo)
            pts += [(x - keep, lo), (x + keep, lo), (x - keep, hi), (x + keep, hi)]
        # Belt and braces for the same failure mode: anchor each side of the
        # hull 2 mm outboard of its top-corner cut line at y0 (clamped to the
        # blank). Both this anchor and the top post's box corner (perp. keep
        # 10 vs the line's 9.5) sit on the OUTBOARD side of the line, and a
        # half plane is convex — so the whole side edge between them stays
        # outboard and the cut line remains the ONLY boundary on its side.
        trimmed = self._top_corner_cuts()
        if trimmed is not None:
            _, cuts, _ = trimmed
            for a, b in cuts.values():
                x_at = a[0] + (y0 - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
                out = 2.0 if x_at >= self.band_mid else -2.0
                x = min(max(x_at + out, -self.plate_half), self.plate_half)
                pts.append((x, y0))
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
        trimmed = self._top_corner_cuts()
        # trim each side back to ITS band edge; on a side whose post keep-box
        # pokes past it, the tab grows out of the band edge through two 45 deg
        # straight shoulders (user sketch — two lines replace the old tangent
        # quarter-rounds). The lower shoulder sits SYMMETRIC to the
        # top-corner cut line's crossing above the boss: the edge margin
        # below the flare ring mirrors the ring's overhang past that
        # crossing (user follow-up sketch), with a keep-clear guard for the
        # ring (2 mm beyond its radius, 45 deg line).
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
            # 45 deg straight shoulders. Junction j: raised from the keep-box
            # bottom until the edge below the boss mirrors the ring's
            # overhang past the top-corner cut line crossing (j = 2*y_post −
            # y_cross); clamped to the keep-box top and to a 2 mm clearance
            # from the flare ring along the 45 deg line.
            y_post = (lo0 + hi0) / 2  # boss center of the poking post
            j = lo0
            if trimmed is not None:
                _, cuts_, _ = trimmed
                a, b = cuts_['left' if sx < 0 else 'right']
                y_cross = a[1] + (x_out - a[0]) * (b[1] - a[1]) / (b[0] - a[0])
                j = max(lo0, min(
                    2 * y_post - y_cross,
                    hi0,
                    y_post + over - (post_d / 2 + 4.5) * math.sqrt(2),
                ))
            pts = [
                (x_in, 0.0),
                (x_in, j - over),
                (x_out, j),
                (x_out, hi0),
                (x_in, hi0 + over),
                (x_in, self.plate_h),
                (x_out, self.plate_h),
                (x_out, 0.0),
            ]
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
        # walks a nut pocket into a line. Line math lives in
        # _top_corner_cuts (shared with _taper_outline, whose hull is kept
        # outboard of these lines — see the anchor note there).
        if trimmed is not None:  # only when the roof is actually tapered
            y0, cuts, normals = trimmed
            top = max(self.all_posts, key=lambda p: p[1])  # H2: top post
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
            # (pocket across-corner radius 4.16 + wall margin, the figure
            # the haunch cap uses). The top/side posts sit at
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
                        f'{cut_guard} mm = nut pocket 4.16 + margin); '
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
            # Soften the four corners the cut lines leave behind (user: the
            # sharp edges feel harsh and the ridge apex is a knock-prone
            # stress point). Each cut face contributes two plate-thickness
            # edges — one at the ridge apex, one at the lower junction with
            # the plate edge. Fillet all four together, backing the radius
            # off if OCC refuses (the two ridge corners can sit close
            # together on narrow caps).
            corner_edges = []
            for _name, (a, b) in cuts.items():
                ux, uy = b[0] - a[0], b[1] - a[1]
                ln = math.hypot(ux, uy)
                corner_edges += [
                    e
                    for e in solid.edges()
                    if abs(e.length - plate_t) < 1e-3
                    and abs(ux * (e.center().Y - a[1])
                            - uy * (e.center().X - a[0])) / ln < 1e-4
                ]
            assert len(corner_edges) == 4, (
                f'expected 4 cut-line corner edges, found {len(corner_edges)}'
            )
            for r in (3.5, 2.5, 1.5):
                try:
                    solid = fillet(corner_edges, r)
                    print(f'note: cut-line corners rounded r={r} (4 edges)')
                    break
                except Exception:
                    continue
            else:
                print('warning: cut-line corner rounding failed — left sharp')
        for hole in self.all_posts:
            x, y = hole
            post = Pos(x, y, plate_t) * Cylinder(
                post_d / 2, self.post_h, align=(Align.CENTER, Align.CENTER, MIN)
            )
            if hole != self.lowest_post or self.flare_supported(hole):
                # 45 deg flare (Ø15 foot ring) where the standoff meets the
                # plate: adhesion-friendly transition, self-supporting.
                # Restored for the lowest post too (user): its ring dips
                # past the plate's bottom edge, and there it prints on the
                # boot's front face — coplanar with the plate front since
                # the buried lip. Only a hypothetical layout whose dipping
                # ring would leave the boot's x-span falls back to a plain
                # cylinder (as the lowest post used to be).
                post += Pos(x, y, plate_t) * Cone(
                    post_d / 2 + 2.5,
                    post_d / 2,
                    1.5,
                    align=(Align.CENTER, Align.CENTER, MIN),
                )
            solid += post
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
                # (M4x12: grip PCB 1.6 + post 3 + wall 2.6 ≈ 7.2 mm + 3.2 nut)
                # comes through the board and pulls the nut against the
                # 2.6 mm front wall the post directly buttresses — load path
                # nut -> wall -> post, pure compression, no printed bridge.
                # No pinch lip (deleted, see nut_af): the 7.2 AF pocket is a
                # light slide fit, so nuts are inserted plate-back-up at PCB
                # assembly and stay by friction until the screw clamps them.
                # The screw passes through the pocket volume; both are cuts,
                # so that is fine.
                hex_r = (nut_af / 2) / math.cos(math.radians(30))
                solid -= Pos(x, y, -EPS) * extrude(
                    RegularPolygon(hex_r, 6, major_radius=True),
                    nut_pocket_depth + EPS,
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
        if self.pad_pockets:
            base -= pad_pockets(self.pad_positions())
        base += joint_keys(self.band_mid)
        base -= joint_screw_bores(self.joint_screw_xs)
        base -= joint_screw_head_pockets(self.joint_screw_xs)
        return base

    # -- upright: backplate + posts + boot foot, one part per tilt angle ------

    def upright_solid(self):
        """The demountable half: everything above the seat plane, printed
        boot-down on the bed (slicer drops the STL's boot face to z=0).
        Layer-by-layer audits (facets + per-layer float) show no bridging
        and no air-start: the raw plate bottom edge is buried inside the
        forward-extended boot, so nothing sweeps over the bed but full
        walls."""
        rad = math.radians(self.tilt)
        body = (
            Pos(0, 0, self.origin_z)
            * Rot(X=90 - self.tilt)
            * self.plate_and_posts()
        )
        # boot + haunch + ribs span the boot footprint: the slab inset by
        # boot_inset per side, so the foot sits fully on the slab's flat top
        # inside the base_top_fillet round-over (the plate band is clamped
        # flush to it). Outboard wall at the joint nut seats is a uniform
        # 1.5 mm — the screws hug the boot edges by construction (see
        # __init__).
        boot_w = self.boot_hi - self.boot_lo
        # Front face = the PLATE's front surface continued down to the seat
        # (user sketch): from the toe at (boot_front, base_t) the face rises
        # parallel to the plate, so the top front edge lands exactly on the
        # plate front face at boot-top height — one uninterrupted surface,
        # no step, constant post_h gap to the PCB. YZ trapezoid, not a box,
        # for that inclined front.
        boot = Pos(self.boot_lo, 0, 0) * extrude(
            Plane.YZ
            * Polygon(
                (self.boot_front, base_t),
                (self.boot_rear, base_t),
                (self.boot_rear, base_t + boot_h),
                (self.boot_front + boot_h * self.st / self.ct,
                 base_t + boot_h),
            ),
            boot_w,
        )
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
        # back-face anchor. Wedge base depth rib_foot (first print wanted
        # the foot slim, but the max-moment root buys more from depth than
        # from thickness): the base rear corner reaches y = rib_foot = 26
        # from the seat origin, and the derived boot rear keeps a 6 mm
        # wall behind it. Apex truncated to rib_top_flat — the zero-width
        # tip printed fragile and adds no stiffness there.
        for rx, rh in [(rx, rib_h) for rx in self.rib_x] + self.stub_ribs:
            boot += Pos(rx - rib_t / 2, 0, base_t) * extrude(
                Plane.YZ * self._wedge(rib_foot, rh, rad, rib_top_flat),
                rib_t,
            )
        # keyboard_drop can park the lowest POCKET post's back-face nut
        # pocket inside the boot's z-band (t30/-35 drop 20: opening spans
        # world z 8.4..15.3 vs boot top 14), leaving the pocket unreachable
        # from behind. Cut an access window through the boot in front of
        # that pocket: the nut slides in with tweezers. Boot-only cut,
        # applied before the union, so the plate's pocket stays intact.
        # With lowest_post_tapped = True the lowest post has a plain
        # self-tap pilot (no pocket, no access needed), so this auto-skips
        # whenever the lowest POCKET clears the boot top on its own (drop
        # 30 / t30/-35: lowest pocket opening z ~ 47 mm).
        lo_x, lo_y = min(self.pocket_posts, key=lambda p: p[1])
        pocket_ac_r = (nut_af / 2) / math.cos(math.radians(30))
        if (lo_y - pocket_ac_r) * self.ct < boot_h + 0.5:
            # rear edge = opening rear (lo_y*st + 4*st) plus nut body + margin
            win_y1 = (lo_y + 4.0) * self.st + 2.0
            boot -= Pos(lo_x - 5.5, self.boot_front, base_t - EPS) * Box(
                11.0, win_y1 - self.boot_front, boot_h + 2 * EPS,
                align=(Align.MIN, Align.MIN, MIN),
            )
        body += boot
        body = self._fillet_rib_seams(body)
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

    def _wedge(self, depth: float, height: float, rad: float,
               top_flat: float = 0.0):
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
        edge lies flat ON it — entirely on the boot, whose front edge now
        reaches forward of the plate's back face (buried lip; first-layer
        material; the face above leans back at tilt from vertical,
        self-supporting). Putting P1 at the perpendicular foot
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
        p1 = (-e / c, 0)
        p3 = (depth, 0)
        p2 = (height * s - e * c, height * c + e * s)
        if 0 < top_flat < depth + e / c - 0.5:
            # truncate the knife apex with a flat ~top_flat wide, parallel
            # to the base (horizontal in print pose): same width would be
            # approached only in the last fractional layers anyway
            zc = p2[1] * (1 - top_flat / (depth + e / c))
            f = zc / p2[1]
            p1t = (p1[0] + (p2[0] - p1[0]) * f, zc)
            p3t = (p3[0] + (p2[0] - p3[0]) * f, zc)
            return Polygon(p1, p3, p3t, p1t, align=None)
        return Polygon(p1, p3, p2, align=None)

    def _rib_seam(self, body, xf):
        """The rib-flank / back-face seam edge at flank x = xf, or None.
        Geometric selection (flank X, long, parallel to the plate's up
        direction (0, st, ct)): the haunch-ramp and boot-ledge junctions
        at the same X fail these tests, as do the top-corner cut edges."""
        for e in body.edges():
            vs = e.vertices()
            if len(vs) != 2:
                continue
            (x1, y1, z1), (x2, y2, z2) = [(v.X, v.Y, v.Z) for v in vs]
            dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
            ln = math.sqrt(dx * dx + dy * dy + dz * dz)
            if ln < 10 or abs(dx) / ln > 0.05:
                continue
            if abs(dy * self.st + dz * self.ct) / ln < 0.99:
                continue
            if abs(x1 - xf) < 0.3 and abs(x2 - xf) < 0.3:
                return e
        return None

    def _fillet_rib_seams(self, body):
        """Concave fillet down each rib flank's seam with the backplate's
        back face: hammering loads concentrate at those vertical corners,
        and the fillet both softens the concentration and adds root
        section. The seams only exist after the boot union, so this runs
        on the merged body. Concave and parallel to the layer stack:
        prints support-free. OCC refuses simultaneous fillets on disjoint
        seams even where each succeeds alone, so they are done one flank
        at a time, backing the radius off on refusal (same policy as the
        cut-line corners); a flank that fails everywhere stays sharp — a
        lost refinement, not a regression."""
        flanks = [
            rx + w
            for rx, _ in [(x, rib_h) for x in self.rib_x] + self.stub_ribs
            for w in (-rib_t / 2, rib_t / 2)
        ]
        radii = []
        for xf in flanks:
            for r in (3.0, 2.5, 2.0, 1.5):
                seam = self._rib_seam(body, xf)
                if seam is None:
                    break  # no seam at this flank (layout change) — skip
                try:
                    body = fillet([seam], r)
                    radii.append(r)
                    break
                except Exception:
                    continue
        if len(radii) == len(flanks):
            rs = '/'.join(f'{r:g}' for r in sorted(set(radii)))
            print(f'note: rib seam fillet r={rs} ({len(radii)} seams)')
        else:
            print(f'warning: rib seam fillet on {len(radii)}/{len(flanks)} '
                  f'seams only — rest left sharp')
        return body

    def _breakaway_flange(self):
        """Thin ring around the boot footprint (--flange): bed-adhesion
        insurance for slick plates. flange_t tall so it snaps off with
        pliers after printing, and clear of every joint feature. Follows
        the boot footprint (the band inset by boot_inset, front edge
        included), so it centers on the band's mid-x too."""
        boot_w = self.boot_hi - self.boot_lo
        outer = RectangleRounded(
            boot_w + 2 * self.flange_w,
            self.boot_rear - self.boot_front + 2 * self.flange_w,
            4,
        )
        inner = Rectangle(boot_w, self.boot_rear - self.boot_front)
        return Pos(
            (self.boot_lo + self.boot_hi) / 2,
            (self.boot_front + self.boot_rear) / 2,
            base_t,
        ) * extrude(outer - inner, flange_t)

    # -- top level ------------------------------------------------------------

    def build(self, part: str, side: str):
        # Handedness: the unmirrored body is currently the RIGHT-hand part
        # (picked at random, no special reason); 'left' mirrors it. The base
        # needs NO
        # mirror: keys, screw bores and the boot flat are all symmetric
        # about the band mid, so the mirrored upright re-seats on the same
        # slab shifted 2*band_mid — one printed slab serves both hands.
        if part == 'base':
            return self.base_solid()
        if part == 'fit':
            body = self.plate_and_posts()
            return mirror(body, about=Plane.YZ) if side == 'left' else body
        if part == 'upright':
            body = self.upright_solid()
            return mirror(body, about=Plane.YZ) if side == 'left' else body
        # 'full': fused assembly preview. For 'left', mount the mirrored
        # upright on the SAME slab the way it is actually assembled:
        # shifted 2*band_mid so grooves/screw pockets land on keys/bores.
        upright = self.upright_solid()
        if side == 'left':
            upright = Pos(2 * self.band_mid, 0, 0) * mirror(
                upright, about=Plane.YZ
            )
        return self.base_solid() + upright


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
        choices=['base', 'upright', 'full', 'fit', 'joint', 'padtest'],
        default='full',
        help="'base' = base slab (follows the band footprint) | "
        "'upright' = per-tilt part | "
        "'full' = fused preview | 'fit' = flat validation plate | "
        "'joint' = fit-test coupons | "
        "'padtest' = slicer-support coupons (pad pocket, head pocket)",
    )
    ap.add_argument(
        '--pad-pockets',
        action='store_true',
        help='cut the 6 pad pockets into the base underside '
        '(slicer-support experiment; default off = plain flat underside)',
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

    if args.part == 'padtest':
        # production-geometry support coupons, no Stand layout needed
        emit(pad_coupon(), 'pad_coupon')
        emit(head_coupon(), 'head_coupon')
        return

    # Set-angles go into the per-layout stems (upright/full/fit): the post
    # pattern (band, ribs, keys) moves with them. The base slab and the
    # joint coupons keep one stable filename, regenerated from the
    # current layout on demand.
    angle_tag = ','.join(f'{a:g}' for a in set_angles)

    if args.part == 'joint':
        # the coupons reuse the exact production features, so they need the
        # derived layout (band mid / screw positions) from a real Stand
        stand = Stand(pcb_w, pcb_h, holes, outline, args.post_h, args.attach,
                      set_angles)
        emit(joint_coupon_base(stand), 'joint_coupon_base')
        emit(joint_coupon_boot(stand), 'joint_coupon_boot')
        return

    # The base is tilt- and side-INDEPENDENT: keys, screw bores and the
    # boot flat are symmetric about the band mid, so one slab serves both
    # hands (the mirrored upright re-seats shifted 2*band_mid).
    if args.part == 'base':
        stands = [
            Stand(
                pcb_w,
                pcb_h,
                holes,
                outline,
                args.post_h,
                args.attach,
                set_angles,
                pad_pockets=args.pad_pockets,
            )
        ]
        stems = ['stand_base' + ('_pads' if args.pad_pockets else '')]
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
                    pad_pockets=args.pad_pockets,
                )
            )
            tag = f'_t{tilt:g}_a{angle_tag}'
            if args.flange:
                tag += f'_flange{args.flange:g}'
            if args.pad_pockets:
                tag += '_pads'
            if args.part == 'upright':
                stems.append(f'stand_{args.side}_upright{tag}')
            elif args.part == 'full':
                stems.append(f'stand_{args.side}_full{tag}')
            else:
                stems.append(f'stand_{args.side}_fit_a{angle_tag}')

    for stand, stem in zip(stands, stems, strict=True):
        for (ang, pts), (_, _, _, corner_z) in zip(
            stand.sets, stand.set_meta, strict=True
        ):
            print(
                f'set {ang:5.1f}°: posts x=[{min(x for x, _ in pts):7.2f}, '
                f'{max(x for x, _ in pts):7.2f}]  '
                f'lowest PCB outline point z={corner_z:6.2f} mm '
                f'(slide margin {(corner_z - stand.origin_z) / stand.ct:.2f} mm)'
            )
        key_xs = [round(stand.band_mid + s * key_x, 1) for s in (-1, 1)]
        print(
            f'ribs at x = {[round(x, 1) for x in stand.rib_x]}'
            + (f' + stubs {[(round(a, 1), round(b, 1)) for a, b in stand.stub_ribs]}'
               if stand.stub_ribs else '')
            + f'  | haunch_h = {stand.haunch_h:.1f} mm'
            f'  | joint: keys x = {key_xs}, '
            f'screws x = {[round(x, 1) for x in stand.joint_screw_xs]} '
            f'({joint_screw} from below)'
        )
        print(
            f'boot: x [{stand.boot_lo:g}, {stand.boot_hi:g}], '
            f'front {stand.boot_front:g}, rear {stand.boot_rear:g} mm'
            f'  | CoM y {stand.com_y:.1f} mm, rear margin '
            f'{stand.boot_rear - stand.com_y:.1f} mm'
        )
        emit(stand.build(args.part, args.side), stem)


if __name__ == '__main__':
    main()
