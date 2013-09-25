/*
  Copyright CNRS-AIST 
  Authors: Florent Lamiraux
*/

#ifndef FLICDISTANCE_H
#define FLICDISTANCE_H

#include "KineoWorks2/kwsMetric.h"

KIT_PREDEF_CLASS(CflicDistance);
KIT_PREDEF_CLASS(CflicSteeringMethod);


/**
   \addtogroup flic
   @{
*/

/**

   \brief Compute an estimation of the length of a flic direct path.

   Derives from from CkwsDistance.
*/

class CflicDistance : public CkwsMetric
{
public:
  
  virtual ~CflicDistance();

  /**
     \brief Return a shared pointer to a distance object

     \param inSteeringMethod To create a direct path, a shared pointer to a steering method is required. This shared pointer is thus given at the construction of CflicDistance. The steering method also indicates whether distance is oriented (see CflicDistance::distance).
  */
  static CflicDistanceShPtr create(const CflicSteeringMethodShPtr inSteeringMethod);

  /**
     \brief Return a shared pointer to a distance object
  */
  static CflicDistanceShPtr createCopy(const CflicDistanceConstShPtr& inDistance);

  /// Clone metric.
  CkwsMetricShPtr clone () const;
		
  /**
     \brief Compute an approximation of the distance between two configurations
     \param inConfig1 first configuration.
     \param inConfig2 second configuration.

     \return Length of direct path if distance is non oriented, HUGE_VAL if distance is oriented and path from first to second configuration goes backward.

     Builds a CflicDirectPath between the configurations and approximate the length of this path by a Simpson integral.

  */
  double distance(const CkwsConfig &inConfig1, const CkwsConfig &inConfig2) const;

  /// Same as distance
  double distanceForSorting(const CkwsConfig& cfg1, const CkwsConfig& cfg2) const;

protected:
  /**
     \brief initialization: store weak pointer to itself
  */
  ktStatus init(const CflicDistanceWkPtr &inWeakPtr);

  /**
     \brief Protected constructor
  */
  CflicDistance(const CflicSteeringMethodShPtr inSteeringMethod);

  /**
     \brief Protected copy constructor
  */
  CflicDistance(const CflicDistance& inDistance);

private:

  /**
     \brief Store weak pointer to object
  */
  CflicDistanceWkPtr attWeakPtr;

  /**
     \brief Store steering method to create direct paths
  */
  CflicSteeringMethodShPtr attSteeringMethod;

  /**
     \brief Whether the steering method is oriented.
  */
  bool attIsOriented;

  /**
     \cond 0
     \brief Count the number of object created and not destroyed
  */
  static unsigned int nbObject;
  /**
     \endcond
  */
};

/**
   @}
*/
#endif
