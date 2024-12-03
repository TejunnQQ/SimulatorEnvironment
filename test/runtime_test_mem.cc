// #include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <complex.h>

#include "runtime_API.h"

#define LEN 10

int main() {
  InitFPGA();

  float _Complex input[LEN], output[LEN];
  for (int i = 0; i < LEN; ++i) {
    input[i] = rand() + rand() * I;
  }

  size_t argsize = 3;

  void *input_addr = rtMalloc(sizeof(float _Complex) * LEN);
  printf("Allocated address = %llx\n", (long long)input_addr);

  rtMemcpyH2D(input, input_addr, sizeof(float _Complex) * LEN);
  rtMemcpyD2H(input_addr, output, sizeof(float _Complex) * LEN);

  rtFree(input_addr);

  float sum = 0.0;
  for (int i = 0; i < LEN; ++i) {
    float diff = cabsf(output[i] - input[i]);
    printf("input[%d] = %f + %fi, output[%d] = %f + %fi\n", 
            i, crealf(input[i]), cimagf(input[i]), i, crealf(output[i]), cimagf(output[i]));
    // EXPECT_EQ(input[i], output[i]);
    sum += diff * diff;
  }
  sum /= LEN;
  printf("MSE = %e\n", sum);
}
