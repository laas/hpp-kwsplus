/*
  Copyright 2007 LAAS-CNRS
  Author: Florent Lamiraux
*/


#include "kwsPlusDirectPath.h"

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

  attUend = privateLength();
  return success;
}


ktStatus CkwsPlusDirectPath::extractFrom(double inParam)
{
  //init
  attUend = privateLength();

  if (CkwsDirectPath::extractFrom(inParam) != KD_OK) {
    cerr  << "ERROR - CkwsPlusDirectPath::extractFrom : CkwsDirectPath::extractFrom DID NOT WORK "   << endl ;
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

  //init
  attUend = privateLength();

  if (CkwsDirectPath::extractTo(inParam) != KD_OK) {
    cout  << "ERROR - CkwsPlusDirectPath::extractTo : CkwsDirectPath::extractTo DID NOT WORK  "   << endl ;
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
    cerr  << "ERROR - CkwsPlusDirectPath::reverse : CkwsDirectPath::reverse DID NOT WORK"  << endl ;
    return KD_ERROR;
  }
  attReverse = !attReverse;
  return KD_OK;
}
