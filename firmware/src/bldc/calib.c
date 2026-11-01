#include "calib.h"
#include <errno.h>

void calib_start(calib_t *c) { *c = (calib_t){0}; }

int calib_step(calib_t *c, float angle, float *uq, float *theta_e) {
    (void)c;
    (void)angle;
    *uq = 0.0f;
    *theta_e = 0.0f;
    // TODO: sweep the field one electrical turn forward then back at ALIGN_V, open loop
    //   dir = sign of the encoder change, fail if it didn't move
    //   hold theta_e = 3*pi/2 at ALIGN_V until settled, zero_elec = electrical angle there
    //   moves the joint up to ~77 deg, tethered at +-90
    return -ENOSYS;
}
