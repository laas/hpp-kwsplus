#include "testProjectionConstraint.h"


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
    if (shootAtDistance(config2, attStandardDeviation) == KD_OK) {
      
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



ktStatus CtestProjectionConstraint::shootAtDistance(CkwsConfig& inOutConfig, double inStandardDeviation)
{
  CkwsConfig randomConfig(attDevice); 
  randomConfig.randomize();

  double dist = inOutConfig.distanceTo(randomConfig);

  if (dist <=0 ) {
    std::cerr << "CtestProjectionConstraint::shootAtDistance: negative distance" << std::endl;
    return KD_ERROR;
  }

  double coef = inStandardDeviation/dist;
  for (unsigned int iDof=0; iDof < attDevice->countDofs(); iDof++) {
    inOutConfig.dofValue(iDof, (1-coef)*inOutConfig.dofValue(iDof) + coef*randomConfig.dofValue(iDof));
  }
  return KD_OK;
}
