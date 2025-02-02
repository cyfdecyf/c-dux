// Footprint for a 6-pin through-hole self-locking top push down switch.
// Available on LCSC: https://item.szlcsc.com/2774931.html
// LCSC id: C2681588.
//
// KiCAD footprint converted with easyeda2kicad with the following command:
//
//     easyeda2kicad --full --lcsc_id=C2681588
//
// The footprint is then adapted to ErgoGen.
//
// Params:
//    show_silk_labels_on_both_sides: default is false
//      If true, the silk labels will be shown on both sides of the PCB.
//    switch_3dmodel_filename: default is '${EASYEDA2KICAD}/easyeda2kicad.3dshapes/SW-TH_5.8ZSPT.wrl'
//      Allows you to specify the path to a 3D model STEP or WRL file to be
//      used when rendering the PCB. Use the ${VAR_NAME} syntax to point to
//      a KiCad configured path.
module.exports = {
  params: {
    designator: 'SW',
    side: 'F',
    show_silk_labels_on_both_sides: false,
    PAD_1: undefined,
    PAD_2: undefined,
    switch_3dmodel_filename: '${EASYEDA2KICAD}/easyeda2kicad.3dshapes/SW-TH_5.8ZSPT.wrl',
  },
  body: p => {
    const silk_labels = (side) => {
  return `
  (fp_text reference "${p.ref}" (at 0 -4) (layer ${side}.SilkS) ${p.ref_hide}
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_line (start 1.40 -0.63) (end 1.02 -0.63) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 1.02 -0.63) (end 1.02 0.76) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 1.02 0.76) (end 1.40 0.76) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -2.73 -2.92) (end -2.92 -2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -0.73 -2.92) (end -1.27 -2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 1.27 -2.92) (end 0.73 -2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 2.73 2.92) (end 2.92 2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 2.92 2.92) (end 2.92 -2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 2.92 -2.92) (end 2.73 -2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start 0.73 2.92) (end 1.27 2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -1.27 2.92) (end -0.73 2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -2.92 -2.92) (end -2.92 2.92) (layer ${side}.SilkS) (width 0.25))
  (fp_line (start -2.92 2.92) (end -2.73 2.92) (layer ${side}.SilkS) (width 0.25))

  (fp_line (start -1.40 -1.02) (end 1.40 -1.02) (layer ${side}.Fab) (width 3.00))
  (fp_line (start 1.40 -1.02) (end 1.40 1.14) (layer ${side}.Fab) (width 3.00))
  (fp_line (start 1.40 1.14) (end -1.40 1.14) (layer ${side}.Fab) (width 3.00))
  (fp_line (start -1.40 -1.02) (end -1.40 -1.02) (layer ${side}.Fab) (width 3.00))
`
    }

    const footprint = `
  ${p.show_silk_labels_on_both_sides ? (silk_labels('F') + silk_labels('B')) : silk_labels(p.side)}

  (fp_text value "SW_Push_Lock" (at 0 4) (layer ${p.side}.Fab)
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_text user %R (at 0 0) (layer ${p.side}.Fab)
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_circle (center 3.05 -3.05) (end 3.08 -3.05) (layer ${p.side}.Fab) (width 0.06))
  (fp_circle (center 2.00 -2.25) (end 2.20 -2.25) (layer ${p.side}.Fab) (width 0.40))
  (fp_circle (center 0.00 -2.25) (end 0.20 -2.25) (layer ${p.side}.Fab) (width 0.40))
  (fp_circle (center -2.00 -2.25) (end -1.80 -2.25) (layer ${p.side}.Fab) (width 0.40))
  (fp_circle (center -2.00 2.25) (end -1.80 2.25) (layer ${p.side}.Fab) (width 0.40))
  (fp_circle (center 0.00 2.25) (end 0.20 2.25) (layer ${p.side}.Fab) (width 0.40))
  (fp_circle (center 2.00 2.25) (end 2.20 2.25) (layer ${p.side}.Fab) (width 0.40))
        
  ${'' /* THT Pads */}
  (pad 1 thru_hole rect (at 2.00 -2.25 0.00) (size 1.52 1.52) (layers *.Cu *.Mask)(drill 0.9999979999999999) ${p.PAD_1.str})
  (pad 2 thru_hole circle (at 0.00 -2.25 0.00) (size 1.52 1.52) (layers *.Cu *.Mask)(drill 0.9999979999999999) ${p.PAD_2.str} )
  (pad 3 thru_hole circle (at -2.00 -2.25 0.00) (size 1.52 1.52) (layers *.Cu *.Mask)(drill 0.9999979999999999))
  (pad 4 thru_hole circle (at -2.00 2.25 0.00) (size 1.52 1.52) (layers *.Cu *.Mask)(drill 0.9999979999999999))
  (pad 5 thru_hole circle (at 0.00 2.25 0.00) (size 1.52 1.52) (layers *.Cu *.Mask)(drill 0.9999979999999999))
  (pad 6 thru_hole circle (at 2.00 2.25 0.00) (size 1.52 1.52) (layers *.Cu *.Mask)(drill 0.9999979999999999))
`

    const model_3d = `
  (model "${p.switch_3dmodel_filename}"
    (offset (xyz 0.000 -0.000 0.000))
    (scale (xyz 1 1 1))
    (rotate (xyz 0 0 180))
  )
`

    const final = `
(module easyeda2kicad:SW-TH_5.8ZSPT (layer ${p.side}.Cu) (tedit 5DC5F6A4)
  ${p.at /* parametric position */}
  (attr through_hole)

  ${footprint}
  ${model_3d}
)
`

    // console.log(final);
    return final;
  }
}