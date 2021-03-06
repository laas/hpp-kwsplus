/*
  Copyright CNRS-AIST 
  Authors: Florent Lamiraux
*/

#include <hpp/kwsplus/direct-path/flic-direct-path.hh>
#include <hpp/kwsplus/direct-path/flic-steering-method.hh>
#include <hpp/kwsplus/direct-path/flic-distance.hh>
#include "flic-manager.hh"

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==3
#define ODEBUG3(x) std::cout << "flicDistance:" << x << std::endl
#define ODEBUG2(x) std::cout << "flicDistance:" << x << std::endl
#define ODEBUG1(x) std::cerr << "flicDistance:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x) 
#define ODEBUG2(x) std::cout << "flicDistance:" << x << std::endl
#define ODEBUG1(x) std::cerr << "flicDistance:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x) 
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "flicDistance:" << x << std::endl
#else
#define ODEBUG3(x) 
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

unsigned int CflicDistance::nbObject = 0;

CflicDistance::~CflicDistance()
{
#if DEBUG==2
  nbObject--;
  std::cout << "number of CflicDistance objects: " << nbObject << std::endl;
#endif
}

CflicDistanceShPtr CflicDistance::create(const CflicSteeringMethodShPtr inSteeringMethod)
{
  CflicDistance* distance = new CflicDistance(inSteeringMethod);
  CflicDistanceShPtr distanceShPtr = CflicDistanceShPtr(distance);
  CflicDistanceShPtr distanceWkPtr = CflicDistanceShPtr(distanceShPtr);

  if (distanceShPtr->init(distanceWkPtr) != KD_OK) {
    distanceShPtr.reset();
  }
  return distanceShPtr;
}


CflicDistanceShPtr CflicDistance::createCopy(const CflicDistanceConstShPtr& inDistance)
{
  CflicDistance* distance = new CflicDistance(*inDistance);
  CflicDistanceShPtr distanceShPtr = CflicDistanceShPtr(distance);
  CflicDistanceShPtr distanceWkPtr = CflicDistanceShPtr(distanceShPtr);

  if (distanceShPtr->init(distanceWkPtr) != KD_OK) {
    distanceShPtr.reset();
  }
  return distanceShPtr;
}

CkwsMetricShPtr CflicDistance::clone () const
{
  return CflicDistance::createCopy (attWeakPtr.lock ());
}

/*
  This function computes an approximation of the length of flicDirectPath between 
  Two configurations. 

  The approximation is obtained by numerically integrating the norm of the derivative of 
  the flat output using Simpson formula.
  At each sample step, the curvature is also computed if the curvature is bigger than the 
  maximal allowed value (in absolute value), the returned distance is a huge number.

*/
double CflicDistance::distance(const CkwsConfig &inConfig1, const CkwsConfig &inConfig2) const
{
  CflicDirectPath path(inConfig1, inConfig2, attSteeringMethod);
  static unsigned int halfNbSamples=20;
  static unsigned int nbSamples=2*halfNbSamples;
  double tabGamma[6];
  double velocity[201];
  double v2 = path.attFlatV2;
  double maxCurvature = path.attMaxCurvature;

  /*
    If the steering method is oriented and v2<=0, then the direct path is
    invalid.
  */
  if ((attIsOriented) &&(v2 <= 0)) {
    ODEBUG2(":distance: oriented and v2 = " << v2);
    return 1e6;
  }

  for (unsigned int iSample=0; iSample<nbSamples+1; iSample++) {
    double u=1.0*iSample/nbSamples;
    CflicManager::flatCombination(&(path.attFlatStartCfg), &(path.attFlatEndCfg),
				  u, v2, 2, tabGamma);
#if DEBUG==3
    double x = tabGamma[0];
    double y = tabGamma[1];
#endif
    double x1 = tabGamma[2];
    double y1 = tabGamma[3];
    double x2 = tabGamma[4];
    double y2 = tabGamma[5];
    double v = sqrt(x1*x1+y1*y1);

    velocity[iSample] = v;
    ODEBUG3("(x, y, x',y', x'',y'') = (" << x << ", " << y << ", " 
	   << x1 << ", " << y1 << ", " << x2 << ", " << y2 << ")");
    ODEBUG3("v = " << v);
    if (v == 0) {
      ODEBUG2(":distance: v = 0");
      return 1e6;
    }
    double curvature = fabs(x1*y2-x2*y1)/pow(v,3.0);
    ODEBUG2("curvature = " << curvature);
    if (curvature >= maxCurvature) {
      ODEBUG2(":distance: curvature = " << curvature << ", max curvature = " << maxCurvature);
      return 1e6;
    }
  }

  /*
    Simpson Formula
  */
  double sum = velocity[nbSamples] - velocity[0];
  ODEBUG2("iSample = 0, v = " << velocity[0]);
  ODEBUG2("iSample = " << nbSamples << ", v = " << velocity[nbSamples]);
  for (unsigned int iSample=0; iSample<halfNbSamples; iSample++) {
    sum += 2*velocity[2*iSample] + 4*velocity[2*iSample+1];
    ODEBUG2("iSample = " << 2*iSample << ", v = " << velocity[2*iSample]);
    ODEBUG2("iSample = " << 2*iSample+1 << ", v = " << velocity[2*iSample+1]);
  }
  double length = sum/(3*nbSamples);
  return length;
}

double CflicDistance::distanceForSorting(const CkwsConfig& cfg1, const CkwsConfig& cfg2) const
{
  return distance (cfg1, cfg2);
}

ktStatus CflicDistance::init(const CflicDistanceWkPtr &inWeakPtr)
{
  ktStatus success = this->CkwsMetric::init(inWeakPtr) ;

  if (KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;
}

CflicDistance::CflicDistance(const CflicSteeringMethodShPtr inSteeringMethod) : 
  attSteeringMethod(inSteeringMethod), 
  attIsOriented(inSteeringMethod->isOriented())
{
#if DEBUG==2
  nbObject++;
  std::cout << "number of CflicDistance objects: " << nbObject << std::endl;
#endif
}

CflicDistance::CflicDistance(const CflicDistance& inDistance) : 
  CkwsMetric (),
  attSteeringMethod(inDistance.attSteeringMethod),
  attIsOriented(inDistance.attIsOriented)
{
#if DEBUG==2
  nbObject++;
  std::cout << "number of CflicDistance objects: " << nbObject << std::endl;
#endif
}
