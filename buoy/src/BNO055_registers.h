#pragma once

#include <cstdint>

// Register definitions and related enums for the BNO055 IMU.  Keeping the
// constants together in a dedicated header makes the driver implementation
// easier to read and prevents accidental duplication when additional
// functionality is added.

namespace BNO055_regs {

    enum Register : uint8_t {
        CHIP_ID    = 0x00,
        PAGE_ID    = 0x07,
        ACC_DATA   = 0x08,
        GYR_DATA   = 0x14,
        QUA_DATA   = 0x20,
        OPR_MODE   = 0x3D,
        PWR_MODE   = 0x3E,
        SYS_TRIGGER= 0x3F,
        // further registers can be added here as needed
    };

    enum PowerMode : uint8_t {
        POWER_NORMAL = 0x00,
        POWER_LOW     = 0x01,
        POWER_SUSPEND = 0x02
    };

    enum OperationMode : uint8_t {
        MODE_CONFIG = 0x00,
        MODE_NDOF   = 0x0C,
        // other operation modes (e.g., IMU, COMPASS, etc.) may be added
    };

}
