#pragma once

#include "stdint.h"

namespace gary_serial {
    typedef union{
        uint8_t u[4];
        float f;
    } uint2float_u;
}
