#ifndef TEST_FLAT_INTER_CART_DP_H
#define TEST_FLAT_INTER_CART_DP_H

#include <iostream>
#include <fstream>
#include <string>

#include "KineoModel/kppDeviceComponent.h"
#include "flicSteeringMethod.h"
#include "flicDirectPath.h"


/**
   \brief Test of direct path and steering method FLatORiENTedINtErpolation.
*/

class CtestFlicDirectPath {
public:
  CtestFlicDirectPath() : fileStream() {};

  /**
     \brief Return a shared pointer to the device.
  */
  const CkppDeviceComponentShPtr device() const {return attKwsDevice;};
  /**
     \brief Return a shared pointer to the steering method.
  */
  const CkwsSteeringMethodShPtr steeringMethod() const {return attSteeringMethod;};
  /**
   \brief Initialization of the object: create a device and a steering method.
  */
  ktStatus init();
  /**
     \brief Plot some mappings and bounding structures.

     Reapeats a given number of times the following operation:
     \li pick two random configurations and build a FLatORiENTedINtErpolation between them,
     \li plot the mapping from default to arc-length parameter and derivative,
     \li plot the mapping from arc-length to default parameter,
     \li plot the upper and lower bounds of \f$\frac{d\gamma}{ds}\f$.
     \li expects an integer in default input flow to let time to see the curves.

     \param nbRndDirectPath number of loops described above.

  */
  ktStatus plotMappingsAndBoundingStruct(unsigned int nbRndDirectPath);
  /**
     \brief Plot Mapping from default param to arc-length param.

     Plot arc-length param vs defaut param in a file.
  */
  void plotMappingDefaultToArcLengthParam(std::string dirName, 
					  CflicDirectPathShPtr flicDirectPath);

  /**
     \brief Plot Mapping from arc-length param to default param.

     Plot defaut param vs arc-length param in a file.
  */
  void plotMappingArcLengthToDefaultParam(std::string dirName, 
					  CflicDirectPathShPtr flicDirectPath);
  /**
     \brief Plot bounds of \f$\frac{d\gamma}{du}\f$
  */
  double plotBoundsDgammaOverDu(std::string dirName, 
				CflicDirectPathShPtr flicDirectPath);
  /**
     \brief Plot bounds of \f$\frac{d^2\gamma}{du^2}\f$
  */
  void plotBoundsD2gammaOverDu2(std::string dirName, 
				CflicDirectPathShPtr flicDirectPath);
  /**
     \brief Plot bounds of \f$\frac{d\gamma}{ds}\f$
  */
  double plotBoundsDgammaOverDs(std::string dirName,
				CflicDirectPathShPtr flicDirectPath);
  /**
     \brief Plot bounds of \f$\frac{d^2\gamma}{ds^2}\f$
  */
  void plotBoundsD2gammaOverDs2(std::string dirName,
				CflicDirectPathShPtr flicDirectPath);


  /**
     \brief Test local method maxAbsoluteDerivative of direct path.
     \param nbRndDirectPath number of random direct paths created.
     \param nbIntervalPerDP number of random interval tested on each direct path.

     Random configurations are produced and a direct path is created between these 
     configurations. If the direct path exists, a random number of intervals are created
     and over each interval, the absolute value of the difference of each dof-value is 
     compared to the product of the maximum derivative of this dof by the interval length. 
     The latter value should be bigger.
  */
  ktStatus testMaxAbsoluteDerivative(unsigned int nbRndDirectPath, 
				     unsigned int nbIntervalPerDP);

  /**
     \brief Test a straight line direct path between two aligned configurations.

     This function is intended to track a bug.
  */
  ktStatus testStraightLineDirectPath();

private:
  /**
     \brief Device with which direct path will be tested.
  */
  CkppDeviceComponentShPtr attKwsDevice;

  /**
     \brief FLatORiENTedINtErpolation steering method.
  */
  CflicSteeringMethodShPtr attSteeringMethod;

  /**
    \brief File stream to write debug information.
  */
  std::ofstream fileStream;

  /**
     \brief create a device and store it in attKwsDevice.
  */
  ktStatus createDevice();
  
  /**
     \brief Create a FLatORiENTedINtErpolation steering method.
  */
  ktStatus createSteeringMethod();
  /**
     \brief Compute the distance on a unit circle
  */
  double distCircle(double theta1, double theta2);
};


#endif

