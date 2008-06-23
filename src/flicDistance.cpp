/*
  Copyright CNRS-AIST 
  Authors: Florent Lamiraux
*/

#include "flicDirectPath.h"
#include "flicDistance.h"
#include "flicManager.h"

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "flicDistance:" << x << std::endl
#define ODEBUG1(x) std::cerr << "flicDistance:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "flicDistance:" << x << std::endl
#else
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

CflicDistanceShPtr CflicDistance::create(const CkwsSteeringMethodShPtr inSteeringMethod)
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
  CflicDirectPath* path = new CflicDirectPath(inConfig1, inConfig2, attSteeringMethod);
  static unsigned int halfNbSamples=20;
  static unsigned int nbSamples=2*halfNbSamples;
  double tabGamma[6];
  double velocity[201];
  double v2 = path->attFlatV2;
  double maxCurvature = path->attMaxCurvature;

  /*
    If the steering method is oriented and v2<=0, then the direct path is
    invalid.
  */
  if ((attIsOriented) &&(v2 <= 0)) {
    delete path;
    return HUGE_VAL;
  }

  for (unsigned int iSample=0; iSample<nbSamples+1; iSample++) {
    double u=1.0*iSample/nbSamples;
    CflicManager::flatCombination(&(path->attFlatStartCfg), &(path->attFlatEndCfg),
				  u, v2, 2, tabGamma);
#if DEBUG==2
    double x = tabGamma[0];
    double y = tabGamma[1];
#endif
    double x1 = tabGamma[2];
    double y1 = tabGamma[3];
    double x2 = tabGamma[4];
    double y2 = tabGamma[5];
    double v = sqrt(x1*x1+y1*y1);

    velocity[iSample] = v;
    ODEBUG2("(x, y, x',y', x'',y'') = (" << x << ", " << y << ", " 
	   << x1 << ", " << y1 << ", " << x2 << ", " << y2 << ")");
    ODEBUG2("v = " << v);
    if (v == 0) {
      delete path;
      return HUGE_VAL;
    }
    double curvature = fabs(x1*y2-x2*y1)/pow(v,3.0);
    ODEBUG2("curvature = " << curvature);
    if (curvature >= maxCurvature) {
      delete path;
      return HUGE_VAL;
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
  delete path;
  return length;
}

ktStatus CflicDistance::init(const CflicDistanceWkPtr &inWeakPtr)
{
  ktStatus success = this->CkwsDistance::init(inWeakPtr) ;

  if (KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;
}

CflicDistance::CflicDistance(const CkwsSteeringMethodShPtr inSteeringMethod) : 
  attSteeringMethod(inSteeringMethod), 
  attIsOriented(inSteeringMethod->isOriented())
{
#if DEBUG==2
  nbObject++;
  std::cout << "number of CflicDistance objects: " << nbObject << std::endl;
#endif
}

CflicDistance::CflicDistance(const CflicDistance& inDistance) : 
  attSteeringMethod(inDistance.attSteeringMethod) 
{
#if DEBUG==2
  nbObject++;
  std::cout << "number of CflicDistance objects: " << nbObject << std::endl;
#endif
}
