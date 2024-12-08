/*****************************************************************************
 * This file is part of uvgVPCCenc V-PCC encoder.
 *
 * Copyright (c) 2024, Tampere University, ITU/ISO/IEC, project contributors
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of the Tampere University or ITU/ISO/IEC nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

#pragma once

#include <cstdint>

// From TMC2, floorLog2() and ceilLog2() are used for V3C and NAL unit size precision calculations
static inline int floorLog2(uint32_t x) {
    if (x == 0) {
        // note: ceilLog2() expects -1 as return value
        return -1;
    }
#ifdef __GNUC__
    return 31 - __builtin_clz(x);
#else
#ifdef _MSC_VER
    unsigned long r = 0;
    _BitScanReverse(&r, x);
    return r;
#else
    int result = 0;
    if (x & 0xffff0000) {
        x >>= 16;
        result += 16;
    }
    if (x & 0xff00) {
        x >>= 8;
        result += 8;
    }
    if (x & 0xf0) {
        x >>= 4;
        result += 4;
    }
    if (x & 0xc) {
        x >>= 2;
        result += 2;
    }
    if (x & 0x2) {
        // x >>= 1;
        result += 1;
    }
    return result;
#endif
#endif
}
static inline int ceilLog2(uint32_t x) { return (x == 0) ? -1 : floorLog2(x - 1) + 1; }

enum ATH_TYPE {
    P_TILE = 0,  // P_TILE (Inter atlas tile
    I_TILE,      // I_TILE (Intra atlas tile)
    SKIP_TILE    // SKIP_TILE (SKIP atlas tile)
};

enum ATDU_PATCH_MODE_I_TILE {
    I_INTRA = 0,    //  0: Non-predicted patch mode
    I_RAW,          //  1: RAW Point Patch mode
    I_EOM,          //  2: EOM Point Patch mode
    I_RESERVED_3,   //  3: I_RESERVED Reserved modes
    I_RESERVED_4,   //  4: I_RESERVED Reserved modes
    I_RESERVED_5,   //  5: I_RESERVED Reserved modes
    I_RESERVED_6,   //  6: I_RESERVED Reserved modes
    I_RESERVED_7,   //  7: I_RESERVED Reserved modes
    I_RESERVED_8,   //  8: I_RESERVED Reserved modes
    I_RESERVED_9,   //  9: I_RESERVED Reserved modes
    I_RESERVED_10,  // 10: I_RESERVED Reserved modes
    I_RESERVED_11,  // 11: I_RESERVED Reserved modes
    I_RESERVED_12,  // 12: I_RESERVED Reserved modes
    I_RESERVED_13,  // 13: I_RESERVED Reserved modes
    I_END           // 14: Patch termination mode
};

enum ATDU_PATCH_MODE_P_TILE {
    P_SKIP = 0,     //  0: Patch Skip mode
    P_MERGE,        //  1: Patch Merge mode
    P_INTER,        //  2: Inter predicted Patch mode
    P_INTRA,        //  3: Non-predicted Patch mode
    P_RAW,          //  4: RAW Point Patch mode
    P_EOM,          //  5: EOM Point Patch mode
    P_RESERVED_6,   //  6: Reserved modes
    P_RESERVED_7,   //  7: Reserved modes
    P_RESERVED_8,   //  8: Reserved modes
    P_RESERVED_9,   //  9: Reserved modes
    P_RESERVED_10,  // 10: Reserved modes
    P_RESERVED_11,  // 11: Reserved modes
    P_RESERVED_12,  // 12: Reserved modes
    P_RESERVED_13,  // 13: Reserved modes
    P_END,          // 14: Patch termination mode
};

enum V3C_UNIT_TYPE {
    V3C_VPS = 0,  //  0: Sequence parameter set
    V3C_AD,       //  1: Patch Data Group
    V3C_OVD,      //  2: Occupancy Video Data
    V3C_GVD,      //  3: Geometry Video Data
    V3C_AVD,      //  4: Attribute Video Data
};

enum NAL_UNIT_TYPE {
    NAL_TRAIL_N = 0,      //  0: Coded tile of a non-TSA, non STSA trailing atlas frame ACL
    NAL_TRAIL_R,          //  1: Coded tile of a non-TSA, non STSA trailing atlas frame ACL
    NAL_TSA_N,            //  2: Coded tile of a TSA atlas frame ACL
    NAL_TSA_R,            //  3: Coded tile of a TSA atlas frame ACL
    NAL_STSA_N,           //  4: Coded tile of a STSA atlas frame ACL
    NAL_STSA_R,           //  5: Coded tile of a STSA atlas frame ACL
    NAL_RADL_N,           //  6: Coded tile of a RADL atlas frame ACL
    NAL_RADL_R,           //  7: Coded tile of a RADL atlas frame ACL
    NAL_RASL_N,           //  8: Coded tile of a RASL atlas frame ACL
    NAL_RASL_R,           //  9: Coded tile of a RASL atlas frame ACL
    NAL_SKIP_N,           // 10: Coded tile of a skipped atlas frame ACL
    NAL_SKIP_R,           // 11: Coded tile of a skipped atlas frame ACL
    NAL_RSV_ACL_N12,      // 12: Reserved non-IRAP sub-layer non-reference ACL NAL unit types ACL
    NAL_RSV_ACL_N14,      // 14: Reserved non-IRAP sub-layer non-reference ACL NAL unit types ACL
    NAL_RSV_ACL_R13,      // 13: Reserved non-IRAP sub-layer reference ACL NAL unit types ACL
    NAL_RSV_ACL_R15,      // 15: Reserved non-IRAP sub-layer reference ACL NAL unit types ACL
    NAL_BLA_W_LP,         // 16: Coded tile of a BLA atlas frame ACL
    NAL_BLA_W_RADL,       // 17: Coded tile of a BLA atlas frame ACL
    NAL_BLA_N_LP,         // 18: Coded tile of a BLA atlas frame ACL
    NAL_GBLA_W_LP,        // 19: Coded tile of a GBLA atlas frame ACL
    NAL_GBLA_W_RADL,      // 20: Coded tile of a GBLA atlas frame ACL
    NAL_GBLA_N_LP,        // 21: Coded tile of a GBLA atlas frame ACL
    NAL_IDR_W_RADL,       // 22: Coded tile of an IDR atlas frame ACL
    NAL_IDR_N_LP,         // 23: Coded tile of an IDR atlas frame ACL
    NAL_GIDR_W_RADL,      // 24: Coded tile of a GIDR atlas frame ACL
    NAL_GIDR_N_LP,        // 25: Coded tile of a GIDR atlas frame ACL
    NAL_CRA,              // 26: Coded tile of a CRA atlas frame ACL
    NAL_GCRA,             // 27: Coded tile of a GCRA atlas frame ACL
    NAL_RSV_IRAP_ACL_28,  // 28: Reserved IRAP ACL NAL unit types ACL
    NAL_RSV_IRAP_ACL_29,  // 29: Reserved IRAP ACL NAL unit types ACL
    NAL_RSV_ACL_30,       // 30: Reserved non-IRAP ACL NAL unit types ACL
    NAL_RSV_ACL_31,       // 31: Reserved non-IRAP ACL NAL unit types ACL
    NAL_RSV_ACL_32,       // 32: Reserved non-IRAP ACL NAL unit types ACL
    NAL_RSV_ACL_33,       // 33: Reserved non-IRAP ACL NAL unit types ACL
    NAL_RSV_ACL_34,       // 34: Reserved non-IRAP ACL NAL unit types ACL
    NAL_RSV_ACL_35,       // 35: Reserved non-IRAP ACL NAL unit types ACL
    NAL_ASPS,             // 36: Atlas sequence parameter set non-ACL
    NAL_AFPS,             // 37: Atlas frame parameter set non-ACL
    NAL_AUD,              // 38: Access unit delimiter non-ACL
    NAL_V3C_AUD,          // 39: V3C access unit delimiter non-ACL
    NAL_EOS,              // 40: End of sequence non-ACL
    NAL_EOB,              // 41: End of bitstream non-ACL
    NAL_FD,               // 42: Filler non-ACL
    NAL_PREFIX_NSEI,      // 43: Non-essential supplemental enhancement information non-ACL
    NAL_SUFFIX_NSEI,      // 44: Non-essential supplemental enhancement information non-ACL
    NAL_PREFIX_ESEI,      // 45: Essential supplemental enhancement information non-ACL
    NAL_SUFFIX_ESEI,      // 46: Essential supplemental enhancement information non-ACL
    NAL_AAPS,             // 47: Atlas adaptation parameter set non-ACL
    NAL_RSV_NACL_48,      // 48:  Reserved non-ACL NAL unit types non-ACL
    NAL_RSV_NACL_49,      // 49: Reserved non-ACL NAL unit types non-ACL
    NAL_RSV_NACL_50,      // 50: Reserved non-ACL NAL unit types non-ACL
    NAL_RSV_NACL_51,      // 51: Reserved non-ACL NAL unit types non-ACL
    NAL_RSV_NACL_52,      // 52: Reserved non-ACL NAL unit types non-ACL
    NAL_UNSPEC_53,        // 53: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_54,        // 54: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_55,        // 55: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_56,        // 56: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_57,        // 57: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_58,        // 58: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_59,        // 59: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_60,        // 60: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_61,        // 61: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_62,        // 62: Unspecified non-ACL NAL unit types non-ACL
    NAL_UNSPEC_63         // 63: Unspecified non-ACL NAL unit types non-ACL
};