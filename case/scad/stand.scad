// ---------------------------------------------------------------------------
// C. Dux — vertical (dodo-style) stand for standing use.
//
// Posture: the board stands upright with its COLUMNS running vertically and
// the controller / thumb edge pointing DOWN, keycaps facing the user. This is
// the standard vertical-split arrangement (cf. Dodo): arms hang down, palms
// face sideways, fingers curl along the columns. It also puts the MCU and
// battery connector at the bottom = low centre of mass, and the encoder knob
// ends up at mid-height facing the user.
//
// Mounting: the bare PCB floats on 4 posts aligned with the board's M4 plated
// holes. Post height (post_h) clears whatever sits on the PCB's back side
// (hotswap sockets ~3-4 mm, solder, connectors) so nothing touches the plate.
//
// Stability: wide base, 3 rear gussets locking the plate to the base,
// 6 rubber-pad recesses on the bottom (self-adhesive pads), and two coin
// slots in the rear face for optional ballast.
//
// Coordinate systems (this trips people up — read before editing `holes`):
//   KiCad: X right, Y *down*. The stand frame is built from the KiCad coords:
//     u (horizontal, from the "top row" long edge)   = Y_k - 75.58
//     v (height above the stand bottom / inner edge) = 154 - (X_k - 128.78)
//   The plate is modelled flat (x = u centred, y = v + bottom_ext, z = out of
//   the front face), then rotated [90 - tilt, 0, 0]: 90 deg stands it up,
//   minus `tilt` leans the top back towards the rear of the base.
//
// Usage:
//   openscad -o stand_full.stl stand.scad
//   openscad -o stand_fit.stl  -D 'part="fit"' stand.scad
//   or open stand.scad in the OpenSCAD GUI and use Window > Customizer.
// ---------------------------------------------------------------------------

/* [Part] */
// Which half of the keyboard this stand is for
side = "left"; // [left, right]
// "full" stand, or flat plate+posts only (quick hole-alignment check)
part = "full"; // [full, fit]
// Translucent PCB ghost + USB marker in the preview (never exported to STL)
show_pcb = true;
// Embossed label on the base front face (empty = none)
label = "C.DUX";

/* [PCB — measured on ergogen/output/pcbs/c-dux.kicad_pcb (left half)] */
// Board bounding box: width along rows x height along columns (mm)
pcb_w = 112.5;
pcb_h = 154.0;
// M4 hole positions [u, v] in board coords (see header for the mapping):
holes = [
    [ 9.06,  26.12],  // H1 (256.66,  84.64) inner top    -> stand bottom left
    [70.96,  26.12],  // H2 (256.66, 146.54) inner bottom -> stand bottom right
    [26.88, 124.71],  // H3 (158.07, 102.46) pinky top    -> stand upper left
    [76.79, 103.98],  // H4 (178.80, 152.37) pinky bottom -> stand upper right
];
// USB cable exit [u, v] — ghost marker only, no geometry depends on it
usb_exit = [18.26, 31.66];

/* [Posture] */
// Recline from vertical in degrees (0 = perfectly upright). Photos of the
// mounted board show the comfortable standing posture at ~20-30 deg recline.
tilt = 20; // [0:1:40]

/* [Backplate] */
plate_t = 6;        // backplate thickness
plate_margin = 10;  // side margin around the PCB outline
bottom_ext = 8;     // plate extends this far below the PCB bottom edge
top_ext = 12;       // plate extends this far above the PCB top edge
plate_r = 8;        // plate corner radius

/* [Posts / fasteners] */
post_h = 5;         // post height = PCB-to-plate gap; clear back-side parts
post_d = 12;        // post diameter
// "nut": M4 hex nut captured in a hex pocket at the post top (recommended)
// "tap": post is threaded by an M4 self-tapping screw into a pilot hole
attach = "nut"; // [nut, tap]
nut_af = 7.4;       // M4 nut across flats + play
nut_h = 3.4;        // M4 nut thickness + play
screw_d = 4.5;      // clearance hole below the nut pocket
tap_d = 3.4;        // pilot hole diameter for self-tapping M4

/* [Base] */
base_t = 10;        // base thickness
base_front = 30;    // depth towards the user (front of the plate line)
base_rear = 65;     // depth behind the plate — main tip-over counterweight
base_side = 18;     // width beyond the PCB on each side
base_r = 10;        // base corner radius

/* [Gussets] */
gussets = true;     // triangular ribs locking the backplate to the base
gusset_x = [-40, 0, 40];
gusset_t = 5;       // gusset thickness
gusset_d = 18;      // gusset length along the base top
gusset_h = 25;      // gusset length up the backplate

/* [Rubber pads (self-adhesive)] */
pad_d = 20;         // pad diameter the recesses are sized for
pad_recess = 0.8;   // alignment recess depth in the bottom face

/* [Coin ballast] */
coin_slot = true;   // slots in the rear face, slide coins in for ballast
coin_x = 24;        // slot centre offset from the middle
coin_w = 27;        // slot width (coin diameter + play)
coin_h = 3.4;       // slot height (coin stack + play)
coin_depth = 14;    // slot depth from the rear face

/* [Quality] */
$fn = 72;

// ---------------------------------------------------------------------------
// Derived dimensions
// ---------------------------------------------------------------------------

eps = 0.01;
embed = 2.5;                    // how deep the plate sinks into the base top
origin_z = base_t - embed;      // plate-local origin height above the desk

plate_w = pcb_w + 2 * plate_margin;
plate_h = bottom_ext + pcb_h + top_ext;

bx = pcb_w + 2 * base_side;     // base width
y0 = -base_front;               // base front edge (towards the user)
y1 = base_rear;                 // base rear edge (behind the plate)

// Rubber pad centres on the bottom face: front row, rear row, mid front/rear
function pad_pos() = let(
    px = bx / 2 - base_r - 4,
    fy = y0 + base_r + 4,
    ry = y1 - base_r - 4
) [[-px, fy], [px, fy], [0, fy], [-px, ry], [px, ry], [0, ry]];

// ---------------------------------------------------------------------------
// Modules — everything below is built in the backplate-local frame
// (x right, y up along the plate, z out of the front face)
// ---------------------------------------------------------------------------

module rrect(sx, sy, r) {
    offset(r = r) offset(delta = -r) square([sx, sy], center = true);
}

module post() {
    cylinder(d = post_d, h = post_h);                    // standoff
    cylinder(d1 = post_d + 5, d2 = post_d, h = 1.5);     // flare for strength
}

module fastener() {
    top = plate_t + post_h;
    if (attach == "nut") {
        // Hex pocket open at the post top: drop the M4 nut in, drive the
        // screw through the PCB into it. Tightening pulls the nut against
        // the pocket floor and clamps the PCB onto the post top face.
        translate([0, 0, top - nut_h])
            cylinder(d = nut_af / cos(30), h = nut_h + eps, $fn = 6);
        cylinder(d = screw_d, h = top - nut_h + eps);    // screw clearance
    } else {
        // Blind pilot hole for an M4 self-tapping screw
        translate([0, 0, 2]) cylinder(d = tap_d, h = top - 2);
    }
}

module plate_and_posts() {
    difference() {
        union() {
            translate([0, plate_h / 2, 0])
                linear_extrude(plate_t) rrect(plate_w, plate_h, plate_r);
            for (h = holes)
                translate([h[0] - pcb_w / 2, h[1] + bottom_ext, plate_t])
                    post();
        }
        for (h = holes)
            translate([h[0] - pcb_w / 2, h[1] + bottom_ext, 0])
                fastener();
    }
}

module pcb_ghost() {
    if (show_pcb) {
        %translate([-pcb_w / 2, bottom_ext, plate_t + post_h])
            cube([pcb_w, pcb_h, 1.6]);
        %for (h = holes)
            translate([h[0] - pcb_w / 2, h[1] + bottom_ext, plate_t + post_h - 0.5])
                cylinder(d = 4.2, h = 3);
        // USB cable exit marker
        %color("red")
            translate([usb_exit[0] - pcb_w / 2 - 4, usb_exit[1] + bottom_ext - 3,
                       plate_t + post_h + 1.6])
                cube([8, 6, 3]);
    }
}

// ---------------------------------------------------------------------------
// Base — built directly in the world frame (desk = XY plane, user at -Y)
// ---------------------------------------------------------------------------

module base_solid() {
    difference() {
        union() {
            // Rounded slab: hull of the four corner cylinders
            hull() for (sx = [-1, 1], cy = [y0 + base_r, y1 - base_r])
                translate([sx * (bx / 2 - base_r), cy, 0])
                    cylinder(r = base_r, h = base_t);
            if (gussets) for (x = gusset_x) gusset(x);
        }
        // Rubber pad alignment recesses in the bottom face
        for (p = pad_pos())
            translate([p[0], p[1], -eps]) cylinder(d = pad_d, h = pad_recess + eps);
        // Coin ballast slots through the rear face
        if (coin_slot) for (sx = [-1, 1])
            translate([sx * coin_x - coin_w / 2, y1 - coin_depth, base_t / 2 - coin_h / 2])
                cube([coin_w, coin_depth + 1, coin_h]);
    }
}

// Triangular rib between the base top and the tilted backplate's back face.
// Polygon (u, v) maps to world (X = const, Y = -2 + u, Z = base_t - 1 + v);
// both anchor corners end up inside the plate slab and the base, so the
// union fuses cleanly.
module gusset(x) {
    translate([x - gusset_t / 2, -2, base_t - 1]) rotate([90, 0, 90])
        linear_extrude(gusset_t)
            polygon([[0, 0], [gusset_d, 0],
                     [gusset_h * sin(tilt), gusset_h * cos(tilt)]]);
}

module base_label() {
    if (len(label) > 0 && part == "full")
        translate([0, y0 + 0.01, 2]) rotate([90, 0, 0])
            linear_extrude(0.9)
                text(label, size = 6, halign = "center");
}

// ---------------------------------------------------------------------------
// Top level
// ---------------------------------------------------------------------------

echo(str("plate: ", plate_w, " x ", plate_h, " mm | base: ", bx, " x ", y1 - y0,
         " mm | height approx.: ", base_t + (plate_h + plate_t) * cos(tilt), " mm"));

module body() {
    if (part == "fit") {
        // Flat plate + posts, lying on the print bed: print this first and
        // offer it up to the PCB to check hole alignment and post clearance.
        plate_and_posts();
        pcb_ghost();
    } else {
        base_solid();
        translate([0, 0, origin_z]) rotate([90 - tilt, 0, 0]) {
            plate_and_posts();
            pcb_ghost();
        }
    }
}

if (side == "right") mirror([1, 0, 0]) body();
else body();
base_label();
