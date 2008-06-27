#ifndef TEST_FLAT_INTER_CART_DP_H
#define TEST_FLAT_INTER_CART_DP_H

#include "KineoModel/kppDeviceComponent.h"
#include "flicSteeringMethod.h"
#include "flicDirectPath.h"
#include "flicDistance.h"

#include "testKwsDirectPath/testKwsDirectPath.h"
/**
   \brief Test of direct path and steering method FLatORiENTedINtErpolation.
*/

class CtestFlicDirectPath : public CtestKwsDirectPath {
public:
  CtestFlicDirectPath() {};

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
     \brief Test a straight line direct path between two aligned configurations.

     This function is intended to track a bug.
  */
  ktStatus testStraightLineDirectPath();

  /**
     \brief Test approximation of direct path length
  */
  ktStatus testApproximateLength(unsigned int nbRndDirectPath);

  /**
     \brief Test velocities computed by CkwsPlusDirectPath::getVelocityAtDistance

     \param inNbRndDirectPath number of random direct paths created
     \param inNbSamplePoints number of sample points where velocity is tested

     Generate random direct paths. For each of them:
     \li compute velocity along the interval of definition and compare with finite-difference,
     \li extract a sub-direct path and perform above test again,
     \li reverse direct path and perform test again.
  */

  ktStatus testDirectPathDeriv(unsigned int inNbRndDirectPath,
			       unsigned int inNbSamplePoints);

private:
  /**
     \brief Approximate distance between config
  */
  CflicDistanceShPtr attFlicDistance;

  /**
     \brief create a device 

     Device is stored in parent class CtestKwsDirectPath.
  */
  ktStatus createDevice();
  
  /**
     \brief Create a FLatORiENTedINtErpolation steering method.

     Steering method is stored in parent class CtestKwsDirectPath.
  */
  ktStatus createSteeringMethod();

  /**
     \brief Create an object that compute an approximation of flic direct path lengths.
  */
  ktStatus createFlicDistance();

  /**
     \brief Compares velocity returned by getVelocityAtDistance and computed by finite-difference

     \param inDirectPath direct path to test
     \param inNbSamplePoints number of sample points where velocity is tested
  */
  ktStatus compareVelocityWithFiniteDif(const CkwsPlusDirectPathShPtr& inDirectPath,
					unsigned int inNbSamplePoints);
};


#endif

