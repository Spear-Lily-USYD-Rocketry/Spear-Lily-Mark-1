
#ifndef UBX_H_
#define UBX_H_

#include "eigen.h"  // NOLINT
#include "ubx_defs.h"  // NOLINT
#include "ubx_nav.h"  // NOLINT

enum Fix {FIX_NONE = 0, FIX_2D = 1, FIX_3D = 2, FIX_DGNSS = 3, FIX_RTK_FLOAT = 4, FIX_RTK_FIXED = 5};

bool Read();

#endif /* UBX_H_ */
