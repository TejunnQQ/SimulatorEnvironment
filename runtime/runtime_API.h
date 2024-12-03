#ifndef TAI_SIM_RUNTIME_API
#define TAI_SIM_RUNTIME_API

#define _API_CALL

typedef void* HANDLE;

void InitFPGA();

unsigned int rtLaunchKernel(unsigned op, size_t argsize, void** args);

void* rtMalloc(size_t size);

void rtFree(void *dev_ptr);

void rtMemcpyH2D(void* dst, void* src, size_t size);

void rtMemcpyD2H(void* dst, void* src, size_t size);

#endif
