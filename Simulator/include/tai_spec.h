#ifndef TAI_SIM_TAI_SPEC_H
#define TAI_SIM_TAI_SPEC_H

/*
*  Device Specification
*/

#include <string>
#include <vector>
#include <memory>
#include <cstring>

namespace tai {
    const std::string  Version        = "0.1.0";
    using ElemType                    = int32_t;
    using Addr                        = uint32_t;
    constexpr uint32_t ElemBytes      = sizeof(ElemType);

    constexpr uint32_t NumCommonRegs  = 256;
    constexpr uint32_t NumSpecRegs    = 256;
    constexpr uint32_t nBytesOfDRAM   = 1 << 28; // 256 MB
    constexpr uint32_t nBytesOfCache  = 1 << 22; //   4 MB

    constexpr uint32_t Batch          = 1;
    constexpr uint32_t BlockIn        = 16;
    constexpr uint32_t BlockOut       = 16;
    constexpr Addr     AccumBase      = 0;
    constexpr Addr     InputBase      = 1 << 20;
    constexpr Addr     ConstBase      = 1 << 21;
    constexpr uint32_t MemIdConst     = 1;
    constexpr uint32_t MemIdInput     = 2;
    constexpr uint32_t MemIdAccum     = 3;
    constexpr uint32_t InputBlock     = Batch * BlockIn;
    constexpr uint32_t ConstBlock     = BlockIn * BlockOut;
    constexpr uint32_t AccumBlock     = Batch * BlockOut;
    constexpr uint32_t InputBlockSize = InputBlock * ElemBytes;
    constexpr uint32_t ConstBlockSize = ConstBlock * ElemBytes;
    constexpr uint32_t AccumBlockSize = AccumBlock * ElemBytes;

    constexpr uint32_t ClockCycle     = 1;
    constexpr uint32_t BasicInstDelay = 1 * ClockCycle;
    constexpr uint32_t AiInstDelay    = 3 * ClockCycle;
    constexpr uint32_t MemRWDelay     = 5 * ClockCycle;



    enum SpecRegNames {
        VERSION,                        // Processor Version
        FFAULTS,
        FTRIM,
        STATUS,
        TASKID,
        TASKCC,
        PROGH,
        PROGL,
        GCCH,
        GCCL,
        TBD,

        RET,                            // Function call return address
        LWGAP,                          // Addr gap of Matrix width
        LWIDTH,                         // Width of a Matrix
        LHGAP,                          // Addr gap of Tensor height
        LHEIGHT,                        // Height of a Tensor
        SWGAP,
        SWIDTH,
        SHGAP,
        SHEIGHT,

        MSIZE,                          // For MMP: width of Data & Result Matrix
        NSIZE,                          // For MMP: length of Weight & Result Matrix
        // For Others: length of Data & Result Matrix
        KSIZE,                          // Length of Data Matrix & width of Weight Matrix
        DWGAP,                          // Address Gap of a row in Data Matrix
        RWGAP,                          // Address Gap of a row in Result Matrix
        PEGRESS,                        // Output port number for MMP
        AEGRESS,                        // Output port number for MMA
        MEGRESS,                        // Output port number for SMM

        ULEN,
        VLEN,
        VSHIFT,

        FWD_TMP,                        // addr for forward data between instructions

        X_PAD_0,
        Y_PAD_0,
        X_PAD_1,
        Y_PAD_1,
        NDIM,
        X_SIZE,
        Y_SIZE,
        Z_SIZE,
        // For permute
        X_AXIS,                         
        Y_AXIS,
        Z_AXIS,
        X_STRIDE,
        RESET_ACC,                      // set 1: reset acc to 0. set 0: accumulate data
        EXTENT,
        ACCUM_OFFSET,
        CONST_OFFSET,
        INPUT_OFFSET,
    };

    enum OutputPorts {
        MMA,
        SMM,
        MCLIP,
    };

    enum class Opcode: int32_t {
        min = 0,
        max = 1,
        add = 2,
        shr = 3,
        sub = 4,
    };
} // namespace tai

#endif //TAI_SIM_TAI_SPEC_H
