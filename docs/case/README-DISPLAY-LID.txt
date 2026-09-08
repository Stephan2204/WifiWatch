ESP32-S3-ETH display lid V2

V1 was invalid because the extracted lid mesh was open/non-watertight and
OpenSCAD dropped it during the boolean operation.

V2 first converts the sliding lid to a closed mesh and only then adds:
- 29.0 x 29.0 mm display window
- rotated display orientation (pin header on left/right long side)
- 39 x 27 mm mounting-hole grid
- four 5 mm mounting bosses
- 1.6 mm M2 pilot holes
- approx. 3.6 mm boss height

This is still a first mechanical test version based on mesh geometry.
