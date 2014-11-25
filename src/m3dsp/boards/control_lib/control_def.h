
#ifndef CONTROL_DEF_H
#define CONTROL_DEF_H

#include "inttypes.h"
#include "ethercat.h"

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define ABS(a)	   (((a) < 0) ? -(a) : (a))
#define SIGN(a)	   (((a) < 0) ? -1 : 1)
#define INC_MOD(a, b)  (((a+1) >= (b)) ? 0 : (a+1))
#define XINC_MOD(a, b, c)  (((a+c) >= (b)) ? a+c-b : (a+c))
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#endif
