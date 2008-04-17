#ifndef TESTPROJECTIONCONSTRAINT_H
#define TESTPROJECTIONCONSTRAINT_H

#include <iostream>
#include "KineoWorks2/kwsConstraint.h"
#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoWorks2/kwsDevice.h"
#include "kwsioConfig.h"


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
     \brief Shoot a random configuration at a given distance to input configuration
  */
  ktStatus shootAtDistance(CkwsConfig& inOutConfig, double inStandardDeviation);

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

#endif
