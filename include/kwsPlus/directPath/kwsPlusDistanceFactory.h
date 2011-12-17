/*
  Copyright CNRS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUSDISTANCEFACTORY_H
#define KWSPLUSDISTANCEFACTORY_H

#include "KineoWorks2/kwsSMLinear.h"
#include "kwsPlus/directPath/reedsSheppSteeringMethod.h"
#include "kwsPlus/directPath/flicDistance.h"
#include "kwsPlus/directPath/kwsPlusDistance.h"
#include "kwsPlus/directPath/flicDistance.h"
#include "kwsPlus/directPath/flicSteeringMethod.h"

/**

\addtogroup distanceFactory
* @{
*/

/**
   \brief Distance function factory

   This abstract class is intended to be derived to build different types of distance functions (CkwsDistance).
*/

class CkwsPlusDistanceFactory {
public:
  virtual ~CkwsPlusDistanceFactory() {};
  /**
     \brief Return a distance function.
  */
  virtual CkwsDistanceShPtr makeDistance(bool inOriented) = 0;
};


/**
   \brief Linear distance function factory
*/
class CkwsPlusLinearDistanceFactory : public CkwsPlusDistanceFactory {
public:

  /**
     \brief Return a distance function associated with the linear steering method
  */
  CkwsDistanceShPtr makeDistance(bool inOriented) {
    return CkwsPlusDistance::create(CkwsSMLinear::create(inOriented));
  };
};


/**
   \brief Reeds and Shepp distance function factory
*/
class CkwsPlusRSDistanceFactory : public CkwsPlusDistanceFactory {
public:
  /**
     \brief Constructor 
     \param inRadius Radius of circular segments
  */
  CkwsPlusRSDistanceFactory(double inRadius) :
    attRadius(inRadius) {
  };

  /**
     \brief Return a distance function associated with Reeds and Shepp steering method
  */
  CkwsDistanceShPtr makeDistance(bool inOriented) {
    return CkwsPlusDistance::create(CreedsSheppSteeringMethod::create(attRadius, inOriented));
  };

private:
  /**
     \brief Radius of circular segments produced by the steering method
  */
  double attRadius;

};


/**
   \brief CflicDistance distance function factory
*/
class CkwsPlusApproxFlicDistanceFactory : public CkwsPlusDistanceFactory {
public:
  /**
     \brief Return a distance function CflicDistance associated with CflicDirectPath
     
     \note CflicDistance computes an approximation of the length of the CflicDirectPath 
     between two configurations.
  */
  CkwsDistanceShPtr makeDistance(bool inOriented) {
    return CflicDistance::create(CflicSteeringMethod::create(inOriented));
  };
};


/**
   @}
*/

#endif
