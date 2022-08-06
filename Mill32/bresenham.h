//
//  bresenham.h
//  Mill32
//
//  Created by Ruedi Heimlicher on 02.08.2022.
//  Copyright © 2022 Ruedi Heimlicher. All rights reserved.
//

#ifndef bresenham_h
#define bresenham_h

#include <stdio.h>


void plot_line (int x0, int y0, int x1, int y1);
void plot_circle (int xm, int ym, int r);
void plot_ellipse_rect (int x0, int y0, int x1, int y1);
void plot_basic_bezier (int x0, int y0, int x1, int y1, int x2, int y2);
void circle (int cx, int cy, int radius);

#endif /* bresenham_h */
