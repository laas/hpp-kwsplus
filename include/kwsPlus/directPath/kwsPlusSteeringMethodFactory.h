/*
  Copyright CNRS-LAAS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUSSTEERINGMETHODFACTORY_H
#define KWSPLUSSTEERINGMETHODFACTORY_H

#include "kwsPlus/directPath/kwsPlusSMLinear.h"
#include "kwsPlus/directPath/flicSteeringMethod.h"

/**

\addtogroup smfactory
* @{
*/

/**
   \brief Steering method factory

   This abstract class is intended to be derived to build different types of steering methods.
*/

class CkwsPlusSteeringMethodFactory {
public:
  virtual ~CkwsPlusSteeringMethodFactory() {};
  /**
     \brief Return a steering method
  */
  virtual CkwsSteeringMethodShPtr makeSteeringMethod(bool inOriented) = 0;
};


/**
   \brief Linear steering method factory
*/
class CkwsPlusLinearSteeringMethodFactory : public CkwsPlusSteeringMethodFactory {
public:

  /**
     \brief Return a linear steering method
  */
  CkwsSteeringMethodShPtr makeSteeringMethod(bool inOriented) {
    std::vector<double> ratioVector;
    return CkwsPlusSMLinear::create(ratioVector, inOriented);
  };
};


/**
   \brief CflicSteeringMethod steering method factory
*/
class CkwsPlusFlicSteeringMethodFactory : public CkwsPlusSteeringMethodFactory {
public:
  /**
     \brief Return a flic steering method
  */
  CkwsSteeringMethodShPtr makeSteeringMethod(bool inOriented) {
    return CflicSteeringMethod::create(inOriented);
  };
};


/**
   @}
*/

#endif
