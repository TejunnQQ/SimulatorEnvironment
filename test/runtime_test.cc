// #include <gtest/gtest.h>
#include <stdio.h>
#include <complex.h>
#include <math.h>

#include "runtime_API.h"

#define LEN 10

int main() {
  InitFPGA();

  float _Complex input1[LEN], input2[LEN], input3[LEN], output[LEN];
  for (int i = 0; i < LEN; ++i) {
    input1[i] = 0.5;
    input2[i] = 2.0 + 3.0 * I;
    input3[i] = 2.5;
  }

  size_t argsize = 3;

  void *input1_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *input2_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *input3_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *vcmul1_output_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *fft_output_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *vcmul2_output_addr = rtMalloc(sizeof(float _Complex) * LEN);
  void *ifft_output_addr = rtMalloc(sizeof(float _Complex) * LEN);

  rtMemcpyH2D(input1, input1_addr, sizeof(float _Complex) * LEN);
  rtMemcpyH2D(input2, input2_addr, sizeof(float _Complex) * LEN);
  rtMemcpyH2D(input3, input3_addr, sizeof(float _Complex) * LEN);

  // vcmul
  void *args[] = {input1_addr, input2_addr, vcmul1_output_addr, (void *)LEN};
  rtLaunchKernel(4, 4 * sizeof(void *), args);

  // fft
  void *args1[] = {vcmul1_output_addr, fft_output_addr, (void *)LEN};
  rtLaunchKernel(1, 3 * sizeof(void *), args1);

  // vcmul
  void *args2[] = {fft_output_addr, input3_addr, vcmul2_output_addr,
                   (void *)LEN};
  rtLaunchKernel(4, 4 * sizeof(void *), args2);

  // ifft
  void *args3[] = {vcmul2_output_addr, ifft_output_addr, (void *)LEN};
  rtLaunchKernel(2, 3 * sizeof(void *), args3);

  rtMemcpyD2H(ifft_output_addr, output, sizeof(float _Complex) * LEN);

  rtFree(input1_addr);
  rtFree(input2_addr);
  rtFree(input3_addr);
  rtFree(vcmul1_output_addr);
  rtFree(fft_output_addr);
  rtFree(vcmul2_output_addr);
  rtFree(ifft_output_addr);

  float sum = 0.0;
  for (int i = 0; i < LEN; ++i) {
    float _Complex expect = input1[i] * input2[i] * input3[i];
    float tem = cabsf(output[i] - expect);
    sum += tem * tem;
    printf("expect[%d] = %f + %fi, \n real[%d] = %f + %fi\n", i, crealf(expect), cimagf(expect), i, crealf(output[i]), cimagf(output[i]));
  }
  sum /= LEN;
  printf("MSE = %e\n", sum);

  // EXPECT_LT(sum, 0.0001);
}
