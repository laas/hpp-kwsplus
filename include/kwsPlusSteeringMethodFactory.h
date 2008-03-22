/*
  Copyright CNRS-LAAS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUSSTEERINGMETHODFACTORY_H
#define KWSPLUSSTEERINGMETHODFACTORY_H

#include "KineoWorks2/kwsSMLinear.h"
#include "reedsSheppSteeringMethod.h"
#include "flicSteeringMethod.h"

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
  /**
     \brief Return a steering method
  */
  virtual CkwsSteeringMethodShPtr makeSteeringMethod() = 0;
};


/**
   \brief Linear steering method factory
*/
class CkwsPlusLinearSteeringMethodFactory : public CkwsPlusSteeringMethodFactory {
public:
  /**
     \brief Constructor 
     \param inOriented Whether steering method is oriented
  */
  CkwsPlusLinearSteeringMethodFactory(bool inOriented=false) : 
    attOriented(inOriented) {
  };

  /**
     \brief Return a linear steering method
  */
  CkwsSteeringMethodShPtr makeSteeringMethod() {
    return CkwsSMLinear::create(attOriented);
  };

private:
  /**
     \brief Whether the steering method is oriented
  */
  bool attOriented;
};


/**
   \brief CreedsSheppSteeringMethod steering method factory
*/
class CkwsPlusRSSteeringMethodFactory : public CkwsPlusSteeringMethodFactory {
public:
  /**
     \brief Constructor 
     \param inRadius Radius of circular segments
     \param inOriented Whether steering method is oriented
  */
  CkwsPlusRSSteeringMethodFactory(double inRadius, bool inOriented=false) :
    attRadius(inRadius),
    attOriented(inOriented) {
  };

  /**
     \brief Return a Reeds and Shepp steering method
  */
  CkwsSteeringMethodShPtr makeSteeringMethod() {
    return CreedsSheppSteeringMethod::create(attRadius, attOriented);
  };

private:
  /**
     \brief Radius of circular segments produced by the steering method
  */
  double attRadius;

  /**
     \brief Whether the steering method is oriented
  */
  bool attOriented;
};


/**
   \brief CflicSteeringMethod steering method factory
*/
class CkwsPlusFlicSteeringMethodFactory : public CkwsPlusSteeringMethodFactory {
public:
  /**
     \brief Constructor 
     \param inOriented Whether steering method is oriented
  */
  CkwsPlusFlicSteeringMethodFactory(bool inOriented=false) : 
    attOriented(inOriented) {
  };

  /**
     \brief Return a flic steering method
  */
  CkwsSteeringMethodShPtr makeSteeringMethod() {
    return CflicSteeringMethod::create(attOriented);
  };

private:
  /**
     \brief Whether the steering method is oriented
  */
  bool attOriented;
};


/**
   @}
*/

#endif
