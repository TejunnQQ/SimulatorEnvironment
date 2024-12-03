//#include <gtest/gtest.h>
#include <stdio.h>
#include <stdlib.h>
#include <complex.h>

#include "runtime_API.h"

#define LEN 10

int main() {
  InitFPGA();

  float _Complex input1[LEN], input2[LEN], output[LEN];
  
  for (int i = 0; i < LEN; ++i) {
    input1[i] = rand() % 1000 + rand() % 1000 * I;
    input2[i] = rand() % 1000 + rand() % 1000 * I;
  }

  size_t argsize = 3;

  void *input1_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *input2_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *output_addr = rtMalloc(sizeof(float _Complex) * LEN);

  rtMemcpyH2D(input1, input1_addr, sizeof(float _Complex) * LEN);
  rtMemcpyH2D(input2, input2_addr, sizeof(float _Complex) * LEN);

  // vcmul
  void *args[] = {input1_addr, input2_addr, output_addr, (void *)LEN};
  rtLaunchKernel(4, 4 * sizeof(void *), args);

  rtMemcpyD2H(output_addr, output, sizeof(float _Complex) * LEN);

  rtFree(input1_addr);
  rtFree(input2_addr);
  rtFree(output_addr);

  float sum = 0.0;
  for (int i = 0; i < LEN; ++i) {
    float _Complex expect = input1[i] * input2[i];
    printf("expect[%d] = %f + %fi, \n real[%d] = %f + %fi\n", i, crealf(expect), cimagf(expect), i, crealf(output[i]), cimagf(output[i]));
    float tem = cabsf(output[i] - expect);
    sum += tem * tem;
  }
  sum /= LEN;
  printf("MSE = %e\n", sum);

  // EXPECT_LT(sum, 0.0001);
}
