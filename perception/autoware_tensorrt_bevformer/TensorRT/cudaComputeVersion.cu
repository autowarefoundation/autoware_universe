#include <cstdio>

int main()
{
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, 0);
  int v = prop.major * 10 + prop.minor;
  printf("%d", v);
  return 0;
}
