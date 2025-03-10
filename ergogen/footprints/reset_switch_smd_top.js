// Footprint for a 4-pad SMD momentary top push down button.
// Available on LCSC: https://item.szlcsc.com/299976.html
// LCSC id: C318884.
//
// KiCAD footprint converted with easyeda2kicad with the following command:
//
//     easyeda2kicad --full --lcsc_id=C318884
//
// The footprint is then adapted to ErgoGen.
//
// Params:
//    reversible: default is false
//      If true, the footprint will be mirrored on the other side of the PCB.
//    switch_3dmodel_filename: default is '${EASYEDA2KICAD}/easyeda2kicad.3dshapes/SW-SMD_4P-L5.1-W5.1-P3.70-LS6.5-TL_H1.5.wrl'
//      Allows you to specify the path to a 3D model STEP or WRL file to be
//      used when rendering the PCB. Use the ${VAR_NAME} syntax to point to
//      a KiCad configured path.

module.exports = {
  params: {
    designator: 'SW',
    side: 'F',
    PAD_1: {type: 'net', value: 'RST'},
    PAD_3: {type: 'net', value: 'GND'},
    reversible: false,
    switch_3dmodel_filename: '${EASYEDA2KICAD}/easyeda2kicad.3dshapes/SW-SMD_4P-L5.1-W5.1-P3.70-LS6.5-TL_H1.5.wrl',
  },
  body: p => {
    const top = `
(module easyeda2kicad:SW-SMD_4P-L5.1-W5.1-P3.70-LS6.5-TL_H1.5
  (layer ${p.side}.Cu) (tedit 5DC5F6A4)
  ${p.at /* parametric position */}
  (attr smd)
`

    const silk_labels = (side) => {
      return `
  (fp_text reference "${p.ref}" (at 0 -5.85) (layer ${side}.SilkS) ${p.ref_hide}
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_line (start -1.00 -1.80) (end 0.90 -1.80) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 0.90 -1.80) (end 1.80 -0.90) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 1.80 -0.90) (end 1.80 0.90) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 1.80 0.90) (end 0.80 1.90) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 0.80 1.90) (end -0.90 1.90) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -0.90 1.90) (end -1.80 1.00) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -1.80 1.00) (end -1.80 -1.00) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -1.80 -1.00) (end -1.00 -1.80) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 1.28 -2.55) (end 2.17 -1.66) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 2.55 -1.17) (end 2.55 1.17) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 2.17 1.66) (end 1.28 2.55) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -2.55 -1.17) (end -2.55 1.17) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -2.17 1.66) (end -1.28 2.55) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -1.28 -2.55) (end -2.17 -1.66) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -1.28 2.55) (end 1.28 2.55) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -1.28 -2.55) (end 1.28 -2.55) (layer ${side}.SilkS) (width 0.25))
  (fp_circle (center 0.00 0.00) (end 1.28 0.00) (layer ${side}.SilkS) (width 0.25))
`
    }

    const bottom = `
  (fp_text value SW-SMD_4P (at 0 5.85) (layer F.Fab)
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_text user %R (at 0 0) (layer F.Fab)
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_circle (center -3.25 -2.55) (end -3.22 -2.55) (layer F.Fab) (width 0.06))

  (model "${p.switch_3dmodel_filename}"
    (offset (xyz 0.000 -0.000 -0.000))
    (scale (xyz 1 1 1))
    (rotate (xyz 0 0 0))
  )
)
`

    const pad = (side) => {
      return `
  (pad 1 smd rect (at -3.00 -1.85 0.00) (size 1.00 0.75) (layers ${side}.Cu ${side}.Paste ${side}.Mask) ${p.PAD_1.str})
  (pad 2 smd rect (at 3.00 -1.85 0.00) (size 1.00 0.75) (layers ${side}.Cu ${side}.Paste ${side}.Mask))
  (pad 3 smd rect (at -3.00 1.85 0.00) (size 1.00 0.75) (layers ${side}.Cu ${side}.Paste ${side}.Mask) ${p.PAD_3.str})
  (pad 4 smd rect (at 3.00 1.85 0.00) (size 1.00 0.75) (layers ${side}.Cu ${side}.Paste ${side}.Mask))
`
    }

    let final = top;

    if(p.side == "F" || p.reversible) {
      final += silk_labels("F");
      final += pad("F");
    }
    if(p.side == "B" || p.reversible) {
      final += silk_labels("B");
      final += pad("B");
    }

    final += bottom;

    // console.log(final);

    return final;
  }
}
