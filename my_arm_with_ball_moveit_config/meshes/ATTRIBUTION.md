# Basket collision mesh

`basket.stl` derives from `meshes/model.obj` in GoogleResearch's
Threshold_Basket_Natural_Finish_Fabric_Liner_Small model, version 1:
https://fuel.gazebosim.org/1.0/GoogleResearch/models/Threshold_Basket_Natural_Finish_Fabric_Liner_Small

Copyright 2020 Google LLC. Licensed under Creative Commons Attribution 4.0:
https://creativecommons.org/licenses/by/4.0/

The OBJ's 33,143 vertex positions and 66,390 triangular faces were converted to
binary STL without scaling, pose changes, decimation, or convexification. STL
stores the original vertex coordinates as 32-bit floats and omits visual
materials/texture coordinates. It retains the source collision surface and
open basket interior. The world model/link/collision transforms are identity;
the world include translates this mesh by (0.75, 0.25, 0.000022) metres.
