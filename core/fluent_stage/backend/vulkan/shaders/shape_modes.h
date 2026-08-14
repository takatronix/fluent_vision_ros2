// shape_modes.h — draw-mode ids shared by shape_mask.frag and the C++
// recorder (GLSL∩C, like the other shared headers). Parameter packing per
// mode is documented in vulkan_renderer.cpp at the fill sites.

const int SM_ROUNDED_RECT = 1;  // pa=(cx,cy,hx,hy) pb=(corner,thickness)
const int SM_CIRCLE = 2;        // pa=(cx,cy,radius,thickness)
const int SM_SEGMENT_ROUND = 3; // pa=(ax,ay,bx,by) pb=(thickness)
const int SM_SEGMENT_BUTT = 4;  // pa=(ax,ay,bx,by) pb=(half_width)
const int SM_ARC = 5;           // pa=(cx,cy,radius,a0) pb=(a1,thickness)
const int SM_TRIANGLE = 6;      // pa=(x0,y0,x1,y1) pb=(x2,y2)
const int SM_GRID = 7;          // pa=(spacing,thickness) pb=(cx,cy,hx,hy)
const int SM_POLYGON = 8;       // pa=(offset,count)
