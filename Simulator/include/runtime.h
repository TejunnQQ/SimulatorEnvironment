#ifndef TAI_SIM_RUNTIME_H
#define TAI_SIM_RUNTIME_H

#ifndef T_DLL
#ifdef _WIN32
#ifdef TVM_EXPORTS
#define T_DLL __declspec(dllexport)
#else
#define T_DLL __declspec(dllimport)
#endif
#else
#define T_DLL __attribute__((visibility("default")))
#endif
#endif
#define _API_CALL
typedef void* HANDLE;







#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stddef.h>

void fpga_open();

unsigned int _API_CALL fpga_set_irq_callback(unsigned int user_irq_num,char* func);
unsigned int _API_CALL fpga_wait_irq(unsigned int user_irq_num, unsigned int timeout, void** args);

//T_DLL void* TAIBufferAlloc(size_t size);
T_DLL void* fpga_alloc_dma(unsigned long long len32, void* buffer, HANDLE share_dma);

/*!
 * \brief Copy data buffer from one location to another.
 * \param from The source buffer base address.
 * \param from_offset The offset of the source buffer.
 * \param to The target buffer base address.
 * \param to_offset The offset of the target buffer.
 * \param size Size of copy.
 * \param kind_mask lowest 2 bits are '01': host->device, '10': device->host

T_DLL void TAIBufferCopy(const void* from, size_t from_offset, void* to, size_t to_offset,
                           size_t size, int kind_mask); */

T_DLL void fpga_send(const void* from, size_t from_offset, HANDLE dma, unsigned long long offset,  unsigned long long len32, 
    unsigned int last, unsigned int mm_addr, unsigned int mm_addr_inc , unsigned int timeout);
	
T_DLL void _API_CALL fpga_recv(const void* from, size_t from_offset, HANDLE dma, unsigned long long offset, unsigned long long len32, 
    unsigned int last, unsigned int mm_addr, unsigned int mm_addr_inc, unsigned int timeout );
//T_DLL void TAIBufferFree(void* ptr);
T_DLL void fpga_free_dma(HANDLE dma);

T_DLL void TLoad(void* src_dram_addr, uint32_t src_elem_offset, uint32_t x_size, uint32_t y_size,
                   uint32_t x_stride, uint32_t x_pad_before, uint32_t y_pad_before,
                   uint32_t x_pad_after, uint32_t y_pad_after, uint32_t dst_sram_index,
                   uint32_t dst_memory_type);

T_DLL void TStore(uint32_t src_elem_offset, uint32_t src_memory_type, void* dst_dram_addr,
                    uint32_t dst_elem_offset, uint32_t x_size, uint32_t y_size, uint32_t x_stride);

T_DLL void* TAIGetPtr(void* buffer);

T_DLL void TAIPushMemRst(uint32_t reset_out, uint64_t dst_index, uint32_t dst_offset,
                           uint64_t src_index, uint32_t src_offset, uint64_t wgt_index,
                           uint32_t wgt_offset);

T_DLL void TAIPushGemmOp(uint32_t reset_out, uint64_t dst_index, uint32_t dst_offset,
                           uint64_t src_index, uint32_t src_offset, uint64_t wgt_index,
                           uint32_t wgt_offset);

T_DLL void TAIPushCuInsts(uint32_t opcode, uint32_t extent, uint32_t reset, uint32_t dst_coeff,
                            uint32_t dst_offset, uint32_t src_coeff, uint32_t src_offset,
                            uint32_t wgt_coeff, uint32_t wgt_offset, uint32_t use_imm, int32_t imm);

T_DLL void TAIInitialize();

T_DLL void TAISynchronize();

T_DLL void TAIPushInst(const char *inst);

T_DLL void TAISetArg(uint64_t val, uint32_t idx);

#ifdef __cplusplus
}
#endif

#endif //TAI_SIM_RUNTIME_H
