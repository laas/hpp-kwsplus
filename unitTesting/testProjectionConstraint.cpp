#include <iostream>
#include "KineoWorks2/kwsConstraint.h"
#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoWorks2/kwsDevice.h"


#if TOYOTA

std::ostream& operator<<(std::ostream& os, const CkwsConfig& config);

std::ostream& operator<<(std::ostream& os, const CkwsConfig& config)
{
  std::vector<double> vec;
  config.getDofValues(vec);
  unsigned int dim = config.size();
  
  for (unsigned int i=0; i < dim; i++) {
    os << vec[i] << " ";
  }
  return os;
}

#else
#include "kwsioConfig.h"
#endif


/**
   \brief Test whether a constraint is a projection over a manifold

   To use this object, you need to
   \li define a device,
   \li define a configuration constraint (CkwsConstraint) for this device ,
   \li add this constraint to the device,
   \li create a CtestProjectionConstraint with input this constraint.

   Then, you can call methods 
   \li CtestProjectionConstraint::continuity to test the continuity of the constraint and
   \li CtestProjectionConstraint::projection to test that the constraint is a projection over a manifold.
*/

class CtestProjectionConstraint
{
public:
  /**
     \brief Constructor taking a reference to a configuration constraint.
  */
  CtestProjectionConstraint(const CkwsConstraintShPtr& inConstraint);

  /**
     \brief test continuity
     \param inNbTest number of random configurations generated

     Generate a given number of random pairs of configurations close to each other.
     Apply the constraint to both configuration and computes the ratio:
     distance between projections over distance between configurations.
  */
  bool continuity(unsigned int inNbTest);

  /**
     \brief Projection
     \param inNbTest number of random configurations generated

     Generate a given number of configurations. For each of them, applies the constraint 
     twice and tests that second application of constraint does not change the configuration.
  */
  bool projection(unsigned int inNbTest);

private:
  /**
     \brief Constraint to be tested.
  */
  CkwsConstraintShPtr attConstraint;
  
  /**
     \brief Device associated to the constraint.
  */
  CkwsDeviceShPtr attDevice;

  /**
     \brief Medium configuration
  */
  CkwsConfig attMediumConfig;

  /**
     \brief Standard deviation 
  */
  double attStandardDeviation;
};


CtestProjectionConstraint::CtestProjectionConstraint(const CkwsConstraintShPtr& inConstraint) :
  attConstraint(inConstraint), attDevice(inConstraint->device()), attMediumConfig(inConstraint->device()),
  attStandardDeviation(1e-6)
{
  attMediumConfig.clear();
}



bool CtestProjectionConstraint::continuity(unsigned int inNbTest)
{
  bool result = true;
  CkwsConfig config1(attDevice), config2(attDevice), proj1(attDevice), proj2(attDevice);

  std::cout << "Testing continuity of constraint " << attConstraint->name() << std::endl;

  //
  // Loop over the number of tests.
  //
  for (unsigned int iTrial=0; iTrial < inNbTest; iTrial++) {
    //
    // Generate first random configuration
    //
    config1.randomize();
    config2 = config1;
    //
    // Generate second configuration close to first one.
    //
    if (CkwsDiffusionShooter::gaussianShoot(config2, attStandardDeviation) == KD_OK) {
      
      //
      // Set device in first configuration: the constraint is applied
      //
      if (attDevice->setCurrentConfig(config1) == KD_OK) {
	//
	// Get first projected configuration
	//
	attDevice->getCurrentConfig(proj1);
	
	//
	// Set device in second configuration: the constraint is applied
	//
	if (attDevice->setCurrentConfig(config2) == KD_OK) {

	  //
	  // Get second projected configuration
	  //
	  attDevice->getCurrentConfig(proj2);

	  //
	  // Compute ratio between distances.
	  //
	  double dConfig = config1.distanceTo(config2);
	  double dProj = proj1.distanceTo(proj2);

	  if (dConfig > 0) {
	    double ratio = dProj/dConfig;
	    if (ratio > 1e4) {
	      result = false;
	    }
	    std::cout << "Config 1: " << config1 << std::endl;
	    std::cout << "Config 2: " << config2 << std::endl;
	    std::cout << "Projection 1: " << proj1 << std::endl;
	    std::cout << "Projection 2: " << proj2 << std::endl;
	    std::cout << "  dProj(q)/dq = " << ratio << std::endl;
	    std::cout << "-----------------------------------------" << std::endl;
	  }
	}
      }
    }
  }
  return result;
}

bool CtestProjectionConstraint::projection(unsigned int inNbTest)
{
  bool result = true;
  CkwsConfig config1(attDevice), config2(attDevice), config3(attDevice);

  std::cout << "Testing that constraint " << attConstraint->name() << " is a projection." << std::endl;

  //
  // Loop over the number of tests.
  //
  for (unsigned int iTrial=0; iTrial < inNbTest; iTrial++) {
    //
    // Generate first random configuration
    //
    config1.randomize();
    
    //
    // Set device in first configuration: the constraint is applied
    //
    if (attDevice->setCurrentConfig(config1) == KD_OK) {
      attDevice->getCurrentConfig(config2);
      
      if (attDevice->setCurrentConfig(config2) == KD_OK) {
	attDevice->getCurrentConfig(config3);
	if (config2.isEquivalent(config3)) {
	  std::cout << " Constraint is a projection." << std::endl;
	}
	else {
	  result = false;
	  std::cout << "  Constraint is not a projection:" << std::endl;
	  std::cout << "    Config2 = " << config2 << std::endl;
	  std::cout << "    Config3 = " << config3 << std::endl;
	  std::cout << "-----------------------------------------" << std::endl;
	}
      }
    }
  }
  return result;
}
