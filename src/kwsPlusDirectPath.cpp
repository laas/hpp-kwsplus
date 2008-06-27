/*
  Copyright 2007 LAAS-CNRS
  Author: Florent Lamiraux
*/


#include "kwsPlusDirectPath.h"

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "CkwsPlusDirectPath:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusDirectPath:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CkwsPlusDirectPath:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

CkwsPlusDirectPath::CkwsPlusDirectPath(const CkwsConfig& inStartConfig, 
				       const CkwsConfig& inEndConfig,
				       const CkwsSteeringMethodShPtr& inSteeringMethod) :
  CkwsDirectPath(inStartConfig, inEndConfig, inSteeringMethod),
  attUstart(0.0),
  attUend(0.0),
  attReverse(false)
{
}

CkwsPlusDirectPath::CkwsPlusDirectPath(const CkwsPlusDirectPath& inDirecPath) :
  CkwsDirectPath(inDirecPath),
  attUstart(inDirecPath.attUstart),
  attUend(inDirecPath.attUend),
  attReverse(inDirecPath.attReverse)
{
}

CkwsPlusDirectPath::~CkwsPlusDirectPath()
{
}

ktStatus CkwsPlusDirectPath::init(const CkwsPlusDirectPathWkPtr& inWeakPtr)
{
  ktStatus success = CkwsDirectPath::init(inWeakPtr) ;

  /* Compute uEnd only if new direct path is not a copy. */
  if (attUend == 0) {
    attUend = privateLength();
  }
  return success;
}


ktStatus CkwsPlusDirectPath::extractFrom(double inParam)
{

  if (CkwsDirectPath::extractFrom(inParam) != KD_OK) {
    //cerr  << "ERROR - CkwsPlusDirectPath::extractFrom : CkwsDirectPath::extractFrom DID NOT WORK "   << endl ;
    return KD_ERROR;
  }

 

  if (attReverse) {
    if ((attUend - inParam) < attUstart) {
      return KD_ERROR;
    }
    attUend = attUend - inParam;
  } else {
    if ((attUstart + inParam) > attUend) {
      return KD_ERROR;
    }
    
    attUstart = attUstart + inParam;
  }

  return KD_OK;
}


ktStatus CkwsPlusDirectPath::extractTo(double inParam)
{

  if (CkwsDirectPath::extractTo(inParam) != KD_OK) {
    //cout  << "ERROR - CkwsPlusDirectPath::extractTo : CkwsDirectPath::extractTo DID NOT WORK  "   << endl ;
    return KD_ERROR;
  }

  if (attReverse) {
    if (inParam > (attUend - attUstart)) {
      return KD_ERROR;
    }
    attUstart = attUend - inParam;
  } else {
    if (inParam < 0) {
      return KD_ERROR;
    }
    attUend = attUstart + inParam;
  }

  return KD_OK;
}


ktStatus CkwsPlusDirectPath::reverse()
{
  if (CkwsDirectPath::reverse() != KD_OK) {
    //cerr  << "ERROR - CkwsPlusDirectPath::reverse : CkwsDirectPath::reverse DID NOT WORK"  << endl ;
    return KD_ERROR;
  }
  attReverse = !attReverse;
  return KD_OK;
}

ktStatus CkwsPlusDirectPath::getVelocityAtDistance(double inParameter, std::vector<double>& outVelocity)
{
  double param;
  if (attReverse) {
    param = attUend - inParameter;
    if (getVelocityAtDistanceAtConstruction(param, outVelocity) != KD_OK) {
      return KD_ERROR;
    }

    for (unsigned int i=0; i<outVelocity.size(); i++) {
      outVelocity[i] *= -1;
    }
  }
  else {
    param = inParameter + attUstart;
    if (getVelocityAtDistanceAtConstruction(param, outVelocity) != KD_OK) {
      return KD_ERROR;
    }
  }
  return KD_OK;
}

ktStatus CkwsPlusDirectPath::getVelocityAtDistanceAtConstruction(double inDistance, 
								 std::vector<double>& outVelocity)
{
  ODEBUG1(":getVelocityAtDistanceAtConstruction not implemented yet.");
  return KD_ERROR;
}
