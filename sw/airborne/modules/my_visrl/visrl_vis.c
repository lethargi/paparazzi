#include "modules/my_visrl/visrl_vis.h"
// #ifdef VIS_AP_H
#include "modules/my_visrl/vis_ap.h"
#include <stdio.h>
// #endif

void visrl_vis_init(void)
{
// #ifdef VIS_NPS_H
// #endif
  printf("THERE");
// #ifdef VIS_AP_H
  listener = cv_add_to_device(&VIS_CAMERA, colorfilter_func, VIS_FPS);
  printf("Inhere");
// #endif
}
