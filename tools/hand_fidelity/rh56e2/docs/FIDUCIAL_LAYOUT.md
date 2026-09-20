# Scale / fiducial layout

`python -m hand_fidelity.rh56e2.fiducial_sheet --out fiducial_A4.svg --png fiducial_A4.png` writes an A4 portrait sheet:
- 10 mm grid over the page, 1 mm rulers along the top and left edges (numbers every 10 mm);
- a 100 mm print-scale check bar at (55..155, 140..146) mm — measure it after printing and record the length;
- a datum cross at (105, 100) mm for aligning the flange face edge or the palm centre line;
- four ArUco DICT_4X4_50 markers (ids 0-3, 30 mm side) centred at (30, 30), (180, 30), (30, 267), (180, 267) mm; when OpenCV is
  not importable the generator draws nested-square fallbacks at the same centres and says so in the JSON.
The PNG path also runs the detector on the raster and writes the detected centres and their error to the JSON (self-check of
the generator, not of a print). Printer scaling is the only thing the check bar catches; a photo's own scale comes from the
marker centres (homography) and the grid.
Placement per view is in PHOTO_GUIDE.md; the sheet must lie in the plane being measured, not behind the object.
