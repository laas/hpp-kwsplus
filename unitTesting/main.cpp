#include "testFlicDirectPath.h"

int main(int argc, char** argv)
{
  CtestFlicDirectPath testFlicDirectPath;
  if (testFlicDirectPath.init() != KD_OK) {
    cerr << "testHppTools: failed to init CtestFlicDirectPath object." << endl;
    return -1;
  }

#if 0
  if (testFlicDirectPath.plotMappingsAndBoundingStruct(100) != KD_OK) {
    return -1;
  }
#endif

  if (testFlicDirectPath.testMaxAbsoluteDerivative(1000, 100) != KD_OK) {
    return -1;
  }
  return 0;
}
