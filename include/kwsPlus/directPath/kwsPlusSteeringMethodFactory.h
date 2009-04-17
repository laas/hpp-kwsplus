/*
  Copyright CNRS-LAAS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUSSTEERINGMETHODFACTORY_H
#define KWSPLUSSTEERINGMETHODFACTORY_H

#include "kwsPlus/directPath/kwsPlusSMLinear.h"
#include "kwsPlus/directPath/reedsSheppSteeringMethod.h"
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
   \brief CreedsSheppSteeringMethod steering method factory
*/
class CkwsPlusRSSteeringMethodFactory : public CkwsPlusSteeringMethodFactory {
public:
  /**
     \brief Constructor 
     \param inRadius Radius of circular segments
  */
  CkwsPlusRSSteeringMethodFactory(double inRadius) :
    attRadius(inRadius) {
  };

  /**
     \brief Return a Reeds and Shepp steering method
  */
  CkwsSteeringMethodShPtr makeSteeringMethod(bool inOriented) {
    return CreedsSheppSteeringMethod::create(attRadius, inOriented);
  };

private:
  /**
     \brief Radius of circular segments produced by the steering method
  */
  double attRadius;

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
