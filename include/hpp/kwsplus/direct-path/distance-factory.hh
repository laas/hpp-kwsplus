/*
  Copyright CNRS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUSDISTANCEFACTORY_H
#define KWSPLUSDISTANCEFACTORY_H

#include "KineoWorks2/kwsSMLinear.h"
#include <hpp/kwsplus/direct-path/flic-distance.hh>
#include <hpp/kwsplus/direct-path/distance.hh>
#include <hpp/kwsplus/direct-path/flic-distance.hh>
#include <hpp/kwsplus/direct-path/flic-steering-method.hh>

/**

\addtogroup distanceFactory
* @{
*/

/**
   \brief Distance function factory

   This abstract class is intended to be derived to build different types of metric functions (CkwsMetric).
*/

class CkwsPlusDistanceFactory {
public:
  virtual ~CkwsPlusDistanceFactory() {};
  /**
     \brief Return a distance function.
  */
  virtual CkwsMetricShPtr makeDistance(bool inOriented) = 0;
};


/**
   \brief Linear distance function factory
*/
class CkwsPlusLinearDistanceFactory : public CkwsPlusDistanceFactory {
public:

  /**
     \brief Return a distance function associated with the linear steering method
  */
  CkwsMetricShPtr makeDistance(bool inOriented) {
    return CkwsPlusDistance::create(CkwsSMLinear::create(inOriented));
  };
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
  CkwsMetricShPtr makeDistance(bool inOriented) {
    return CflicDistance::create(CflicSteeringMethod::create(inOriented));
  };
};


/**
   @}
*/

#endif
