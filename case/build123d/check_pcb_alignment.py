"""One-shot cross-check: fabricated drill file <-> KiCad design <-> stand.py.

  gerber PTH.drl  --(coords equal)-->  KiCad footprint 'at'
  KiCad footprint --(parse_pcb + HOLES_EXPECTED)--> stand frame holes
  stand frame holes --(solver rigid transform)--> post positions
"""
import math
import re
import sys
import tempfile
import zipfile

from itertools import permutations
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent))
import stand


PCB = Path(__file__).parents[2] / 'pcb' / 'v2.0' / 'c-dux.kicad_pcb'
ZIP = Path(__file__).parents[2] / 'pcb' / 'v2.0' / 'c-dux-gerbers.zip'
TOL_MM = 0.05

text = PCB.read_text()

# -- 1. parse_pcb: HOLES_EXPECTED assertion runs inside --------------------
pcb_w, pcb_h, holes, outline = stand.parse_pcb(PCB)
print(f'[1] KiCad -> HOLES_EXPECTED ... OK (tol {stand.HOLE_TOL} mm)')
for ref, (u, v) in zip(sorted(HOLES_EXPECTED := stand.HOLES_EXPECTED), holes, strict=False):
    print(f'    {ref}: u={u:8.3f}  v={v:8.3f}')

# -- edge extremes, same filter as parse_pcb -------------------------------
xs, ys = [], []
for line in text.splitlines():
    if 'Edge.Cuts' in line and line.strip().startswith('(gr_'):
        for a, b in re.findall(
            r'\((?:start|end|center)\s+(-?[\d.]+)\s+(-?[\d.]+)', line
        ):
            xs.append(float(a))
            ys.append(float(b))
x_min, y_max = min(xs), max(ys)

# -- 2. footprint drill size + at coords -----------------------------------
fps = {}
for chunk in text.split('(footprint "ceoloide:mounting_hole_plated"')[1:]:
    head = chunk[:1500]
    at = re.search(r'\(at\s+(-?[\d.]+)\s+(-?[\d.]+)', head)
    ref = re.search(r'\(property "Reference"\s+"([^"]+)"', head)
    if at and ref:
        fps[ref.group(1)] = (float(at.group(1)), float(at.group(2)))
drill_d = float(
    re.search(r'mounting_hole_plated".*?\(drill\s+([\d.]+)', text, re.S)
    and re.findall(
        r'\(pad "" thru_hole circle[^\n]*?\(drill\s+([\d.]+)', text
    )[0]
)
print(f'[2] footprints: {fps}  drill Ø{drill_d}')

# -- 3. Excellon: find the Ø-drill tool hits in the fabricated board -------
def parse_drl(txt: str) -> dict[float, list[tuple[float, float]]]:
    tools: dict[str, float] = {}
    hits: dict[float, list[tuple[float, float]]] = {}
    cur = None
    for ln in txt.splitlines():
        m = re.match(r'^T(\d+)C([\d.]+)', ln)
        if m:
            tools[m.group(1)] = float(m.group(2))
            continue
        m = re.match(r'^T(\d+)\s*$', ln)
        if m:
            cur = tools.get(m.group(1))
            continue
        m = re.match(r'^X(-?[\d.]+)Y(-?[\d.]+)', ln)
        if m and cur is not None:
            hits.setdefault(cur, []).append(
                (float(m.group(1)), float(m.group(2)))
            )
    return hits

with tempfile.TemporaryDirectory() as td:
    with zipfile.ZipFile(ZIP) as z:
        z.extractall(td)
    td = Path(td)
    pth = parse_drl((td / 'c-dux-routed-PTH.drl').read_text())
    npth = parse_drl((td / 'c-dux-routed-NPTH.drl').read_text())
    gm1 = (td / 'c-dux-routed-Edge_Cuts.gm1').read_text()

match = [d for d in pth if abs(d - drill_d) < 0.01]
assert match, f'no Ø{drill_d} tool in PTH.drl: {sorted(pth)}'
drill_hits = pth[match[0]]
assert len(drill_hits) == 4, f'expected 4 hits, got {len(drill_hits)}'
assert not any(abs(d - drill_d) < 0.01 for d in npth), (
    'Ømounting tool also in NPTH (should be plated-only)'
)
print(f'[3] PTH.drl Ø{match[0]}: {len(drill_hits)} hits, NPTH clean')

# -- 4. drill coords vs footprint at coords -------------------------------
# KiCad drill export flips the Y sign (Y-down internal frame): X stays,
# Y_drill = -Y_footprint.
worst = 0.0
for _ref, (fx, fy) in fps.items():
    d = min(math.hypot(hx - fx, hy + fy) for hx, hy in drill_hits)
    worst = max(worst, d)
assert worst < TOL_MM, f'drill vs footprint worst {worst:.4f} mm'
print(f'[4] drill hits vs footprint at: worst {worst:.4f} mm ... OK')

#    ... and through the stand-frame transform
worst = 0.0
for ref, (u, v) in zip(sorted(fps), holes, strict=False):
    fx, fy = fps[ref]
    d = math.hypot((y_max - fy) - u, (fx - x_min) - v)
    worst = max(worst, d)
assert worst < TOL_MM, f'transform check worst {worst:.4f} mm'
print(f'[4b] KiCad->stand transform of drill hits: worst {worst:.4f} mm ... OK')

# -- 5. Edge.Cuts gerber extents vs KiCad outline bbox ---------------------
gx, gy = [], []
for ln in gm1.splitlines():
    if '%' in ln:  # skip %FS/%MO extended commands (FSLAX46Y46 masquerades as X/Y)
        continue
    for m in re.finditer(r'X(-?\d+)Y(-?\d+)', ln):
        gx.append(int(m.group(1)) / 1e6)
        gy.append(int(m.group(2)) / 1e6)
gw, gh = max(gy) - min(gy), max(gx) - min(gx)
print(f'[5] gerber outline {gw:.3f} x {gh:.3f} mm  vs  kicad {pcb_w:.3f} x {pcb_h:.3f}')
assert abs(gw - pcb_w) < TOL_MM and abs(gh - pcb_h) < TOL_MM, (
    'outline extents mismatch'
)
print('    outline extents ... OK')

# -- 6. posts are a rigid image of the hole pattern ------------------------
s = stand.Stand(pcb_w, pcb_h, holes, outline, 9, 'nut', [-40])
posts = s.all_posts
refs = sorted(fps)
print(f'[6] posts (stand local): {[(round(x,2), round(y,2)) for x, y in posts]}')

# solver output order is not the parse order -> find the 1:1 mapping by
# brute force over 24 permutations, then compare all pairwise distances
def shoelace(pts):
    return sum(
        pts[i][0] * pts[(i + 1) % len(pts)][1]
        - pts[(i + 1) % len(pts)][0] * pts[i][1]
        for i in range(len(pts))
    )


best_perm, best_mm = None, 1e9
for perm in permutations(range(4)):
    mm = max(
        abs(math.dist(posts[i], posts[j])
            - math.dist(holes[perm[i]], holes[perm[j]]))
        for i in range(4)
        for j in range(i + 1, 4)
    )
    if mm < best_mm:
        best_perm, best_mm = perm, mm
assert best_mm < 1e-6, f'no rigid 1:1 mapping (best {best_mm:.4f} mm)'

# same signed area -> the mapping is a rotation, not a mirror
a_holes = shoelace([holes[i] for i in best_perm])
a_posts = shoelace(posts)
assert a_holes * a_posts > 0 and abs(abs(a_holes) - abs(a_posts)) < 1e-6, (
    'pattern mirrored, not rotated'
)
print('    post<->PCB hole mapping:',
      {f'({x:.2f},{y:.2f})': refs[p] for (x, y), p in zip(posts, best_perm, strict=False)})
print(f'    rigid, no mirror (max dev {best_mm:.2e} mm) ... OK')
low = min(posts, key=lambda p: p[1])
low_ref = refs[best_perm[posts.index(low)]]
print(f'    lowest post {low} <- {low_ref} '
      f'(lowest_post_tapped targets it) ... OK')

print('\nALL CHECKS PASSED')
