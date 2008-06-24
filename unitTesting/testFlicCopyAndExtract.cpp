#include "KineoModel/kppLicense.h"
#include "testFlicDirectPath.h"

int main(int argc, char** argv)
{
  if(!CkppLicense::initialize()) {
    std::cerr << "Error: license for KPP SDK was not found" << std::endl;
    return -1;
  }
  else {
    std::cout << "license for KPP SDK was found" << std::endl;
  }

  CtestFlicDirectPath testFlicDirectPath;
  if (testFlicDirectPath.init() != KD_OK) {
    cerr << "testFlicarDirectPath: failed to init CtestFlicarDirectPath object." << endl;
    return -1;
  }

  if (testFlicDirectPath.testCopyExtractReverse(1000, 100, 100) != KD_OK) {
    return -1;
  }
  return 0;
}
