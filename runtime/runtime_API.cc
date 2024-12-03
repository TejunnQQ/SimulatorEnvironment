#include <iostream>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <map>

#include <algorithm>
#include <math.h>
#include <complex.h>
#include "/data/zxy/test/Environment/Simulator/include/runtime.h"
#include "/data/zxy/test/Environment/Simulator/include/tai_sim.h"

#define _API_CALL
#include <cstdarg>

typedef void* HANDLE;

using namespace std;
    
void* rtMalloc(size_t size) {
    return fpga_alloc_dma(size,NULL,NULL);
}

void rtFree(void *dev_ptr) {
    fpga_free_dma(dev_ptr);
}

void rtMemcpyH2D(void* dst, void* src, size_t size) {
    fpga_send(dst, 0, src, 0, size,0,1,1,1);   
}

void rtMemcpyD2H(void* dst, void* src, size_t size) {
    fpga_recv(dst, 0, src, 0, size, 0, 1, 1, 1);
}

void InitFPGA() {
    fpga_open();
}

unsigned int rtLaunchKernel(unsigned int op, size_t argsize, void** args) {
    while (op) {
        fpga_wait_irq(op, 2, args);
        return 0;
    }
}
