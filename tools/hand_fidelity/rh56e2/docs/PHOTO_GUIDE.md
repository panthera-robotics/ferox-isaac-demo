# RH56E2 photo guide (scale-referenced views for geometry extraction)

Rules: the fiducial sheet (`fiducial_A4.svg`, printed at 100 %, check bar measured) lies in the plane you are measuring;
the camera axis is as normal to that plane as possible (all four markers visible, none cut); diffuse light, no flash on
the pads; a second photo of every view from a different angle (the two-photo triangulation gives hinge positions out of
plane); file names `<side>_<view>_<n>.jpg` listed in the sheet's `source_evidence` with sha256; nameplate/serial photos in a
separate private folder, never in the public repository. Write the printed check-bar length on the first photo of the set.

| view | plane / sheet placement | what must be visible | feeds |
|---|---|---|---|
| V1 palm | sheet parallel to the palm face (D2), flange edge on the datum cross | four finger MCP hinges, thumb base, palm pad, wrist flange | hinge centres (x, y), palm width, pad outlines |
| V2 back | sheet parallel to the back of the hand | dorsal hinge screws, nail arrays | hinge check, tactile array positions |
| V3 thumb side (radial) | sheet in the plane containing the index finger and the thumb rotation axis | thumb rotation axis, thumb MP/IP hinges, palm thickness at the pocket | thumb axis point/direction, palm thickness |
| V4 little side (ulnar) | sheet in the sagittal plane of the little finger | MCP/PIP hinge stack, flange face | hinge z (out-of-palm) coordinates, flange-to-MCP row |
| V5 finger open (per finger) | sheet in the finger's flexion plane (normal to its MCP axis), command 1000 | proximal phalanx and the metacarpal plane edge | alpha open (protractor overlay), tip position open |
| V6 finger closed (per finger) | same plane, command 0 | same | alpha closed, tip position closed |
| V7 thumb bend open/closed | plane parallel to the thumb rotation axis (Figure 5 theta) | thumb distal segment and the axis line | theta open/closed |
| V8 thumb rotation open/closed | plane normal to the thumb rotation axis (Figure 5 beta) | thumb line and the metacarpal plane edge | beta open/closed, rotation sense |
| V9 wrist mount | sheet against the adapter plate | flange, adapter, `wrist_yaw_link` reference edges | flange-to-wrist_yaw transform |
| V10 mass/COM | sheet vertical behind the suspended hand | plumb line, attachment point, hand outline | COM by suspension |
| V11 nameplate (PRIVATE) | any | model code, serial, variant | serial_firmware_sheet |

Protractor overlay: measure the angle in the photo between the two lines named in manual Figure 5 (alpha: proximal phalanx
vs the metacarpal plane; theta: thumb distal segment vs a line parallel to the rotation axis; beta: thumb line vs the
metacarpal plane in the plane normal to the axis). Declare ±2° unless the photo geometry justifies less.
Scale: use the marker centres (30 mm markers, centres in `fiducial_A4.json`) for a homography of the sheet plane; the mm
grid is the visual check. Out-of-plane points are not measurable from one photo — that is what the second angle is for.
