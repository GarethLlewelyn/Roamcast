#pragma once

#include "../roamcast/RoamCastConfig.h"

namespace M5AtomEchoS3R {
    // Returns a fully configured RoamCastConfig for the M5Stack Atom EchoS3R.
    // All callbacks are wired to M5Unified. isFullDuplex = false.
    // All features enabled. I2C pins 2/1 (Grove port).
    RoamCastConfig config();
}
