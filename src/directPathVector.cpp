/*
  Copyright 2007 Florent Lamiraux
*/

#include <iostream>
#include "directPathVector.h"

void CdirectPathVector::computeMaxOfTwoVectors(std::vector<double>& inOutVector1, 
					       const std::vector<double>& inVector2) const
{
  for (unsigned int i=0; i<std::max(inOutVector1.size(), inVector2.size()); i++) {
    inOutVector1[i] = std::max(inOutVector1[i], inVector2[i]);
  }
}

CdirectPathVectorShPtr CdirectPathVector::create(const CkwsConfig& inStartCfg, 
						 const CkwsConfig& inEndCfg,
						 const CkwsSteeringMethodShPtr& inSteeringMethod,
						 const std::vector<CkwsDirectPathConstShPtr>& inDirectPathVector) 
{
  CdirectPathVector* DPVectorPtr = new CdirectPathVector(inStartCfg, inEndCfg, inSteeringMethod, inDirectPathVector);
  CdirectPathVectorShPtr DPVectorShPtr(DPVectorPtr);
  CdirectPathVectorWkPtr DPVectorWkPtr(DPVectorShPtr);

  if (DPVectorPtr->init(DPVectorWkPtr) != KD_OK) {
    DPVectorShPtr.reset();
  }
  return DPVectorShPtr;
}


CkwsAbstractPathShPtr CdirectPathVector::clone() const
{
  return CdirectPathVector::createCopy(attWeakPtr.lock());
}


CdirectPathVectorShPtr CdirectPathVector::createCopy(const CdirectPathVectorConstShPtr &inDirectPathVector)
{
  if (inDirectPathVector) {
    CdirectPathVector* DPVectorPtr = new CdirectPathVector(*inDirectPathVector);
    CdirectPathVectorShPtr DPVectorShPtr(DPVectorPtr);
    CdirectPathVectorWkPtr DPVectorWkPtr(DPVectorShPtr);

    if (DPVectorPtr->init(DPVectorWkPtr) != KD_OK) {
      DPVectorShPtr.reset();
    }
    return DPVectorShPtr;
  }
  return CdirectPathVectorShPtr();
}


CdirectPathVector::CdirectPathVector(const CkwsConfig& inStartConfig, 
				     const CkwsConfig& inEndConfig,
				     const CkwsSteeringMethodShPtr& inSteeringMethod,
				     std::vector<CkwsDirectPathConstShPtr> inDirectPathVector) :
  CkwsDirectPath(inStartConfig, inEndConfig, inSteeringMethod), 
  attDirectPathVector(inDirectPathVector),
  attUstart(0.0),
  attUend(1.0),
  attReverse(false)
{
  attLength = 0;
  //Compute length
  for (unsigned int iDP=0; iDP < inDirectPathVector.size(); iDP++) {
    if (inDirectPathVector[iDP]) {
      attLength += inDirectPathVector[iDP]->length();
    }
  }
}


CdirectPathVector::CdirectPathVector(const CdirectPathVector& inDirectPathVector) :
  CkwsDirectPath(inDirectPathVector), 
  attDirectPathVector(inDirectPathVector.attDirectPathVector),
  attLength(inDirectPathVector.attLength),
  attUstart(inDirectPathVector.attUstart),
  attUend(inDirectPathVector.attUend),
  attReverse(inDirectPathVector.attReverse)

{
}

ktStatus CdirectPathVector::init(const CdirectPathVectorWkPtr& inWeakPtr)
{
  ktStatus success = CkwsDirectPath::init(inWeakPtr) ;

  // Check that vector is not empty
  CdirectPathVectorShPtr directPath = inWeakPtr.lock();
  if (directPath->attDirectPathVector.size() == 0) {
    std::cerr << "CdirectPathVector::init: vector of direct paths is empty." << std::endl;
    success = KD_ERROR;
  }

  if (KD_OK == success) {
    // Check that shared pointers in vector are not NULL
    for (unsigned int iDP=0; iDP < directPath->attDirectPathVector.size(); iDP++) {
      if (!directPath->attDirectPathVector[iDP]) {
	std::cerr << "CdirectPathVector::init: a direct path in vector is NULL." << std::endl;
	success = KD_ERROR;
      }
    }
  }

  if (KD_OK == success) {
    attWeakPtr = inWeakPtr;
  }

  return success;

}

void CdirectPathVector::interpolate(double inLength, CkwsConfig& outConfig) const
{
  // Find direct path in vector corresponding to parameter.
  unsigned int iDP=0;
  bool finished = false;

  while ((iDP < attDirectPathVector.size()-1) && !finished) {
    if (inLength > attDirectPathVector[iDP]->length()) {
      inLength -= attDirectPathVector[iDP]->length();
      iDP++;
    }
    else {
      finished = true;
    }
  }
  // 
  if (inLength > attDirectPathVector[iDP]->length()) {
    if (iDP != attDirectPathVector.size()-1) {
      std::cerr << "CdirectPathVector::interpolate: this should not happen" << std::endl;
      return;
    }
    inLength = attDirectPathVector[iDP]->length();
  }
  CkwsDirectPathConstShPtr directPath = attDirectPathVector[iDP];
  directPath->getConfigAtDistance(inLength, outConfig);
}

void CdirectPathVector::maxAbsoluteDerivative(double inFrom, double inTo, 
					      std::vector<double>& outDerivatives) const
{
  if (inFrom > inTo) {
    return;
  }
  if (inFrom < 0) {
    inFrom = 0;
  }

  // Find direct path corresponding to parameter inFrom
  unsigned int iDPFrom=0;
  bool finished = false;
  while ((iDPFrom < attDirectPathVector.size()-1) && !finished) {
    if (inFrom > attDirectPathVector[iDPFrom]->length()) {
      inFrom -= attDirectPathVector[iDPFrom]->length();
      inTo -= attDirectPathVector[iDPFrom]->length();
      iDPFrom++;
    } 
    else {
      finished = true;
    }
  }

  // If first parameter is bigger than upper bound of definition interval, return.
  if (inFrom > attDirectPathVector[iDPFrom]->length()) {
    return;
  }

  // Find direct path corresponding to  parameter inTo
  unsigned int iDPTo=iDPFrom;
  finished = false;

  while ((iDPTo < attDirectPathVector.size()-1) && !finished) {
    if (inTo > attDirectPathVector[iDPTo]->length()) {
      inTo -= attDirectPathVector[iDPTo]->length();
      iDPTo++;
    }
    else {
      finished = true;
    }
  }
  if (inTo > attDirectPathVector[iDPTo]->length()) {
    inTo = attDirectPathVector[iDPTo]->length();
  }
  
  // iDPFrom, inFrom represent the index of and parameter on direct path corresponding to first value.
  // iDPTo, inTo represent the index of and parameter on direct path corresponding to second value.
  if (iDPFrom == iDPTo) {
    return attDirectPathVector[iDPTo]->maxAbsoluteDerivative(inFrom, inTo, outDerivatives);
  }
  
  attDirectPathVector[iDPFrom]->maxAbsoluteDerivative(inFrom, attDirectPathVector[iDPFrom]->length(), 
						     outDerivatives);

  unsigned int iDP=iDPFrom+1; 
  std::vector<double> maxDerivative;

  while (iDP<iDPTo) {
    attDirectPathVector[iDP]->maxAbsoluteDerivative(0, attDirectPathVector[iDPFrom]->length(), 
						   maxDerivative);
    computeMaxOfTwoVectors(outDerivatives, maxDerivative);
    iDP++;
  }
  attDirectPathVector[iDP]->maxAbsoluteDerivative(0, inTo, maxDerivative);
  computeMaxOfTwoVectors(outDerivatives, maxDerivative);
  
}

ktStatus CdirectPathVector::extractFrom(double inParam)
{
  if (CkwsDirectPath::extractFrom(inParam) != KD_OK) {
    return KD_ERROR;
  }

  if (attReverse) {
    if (attUend - inParam < attUstart) {
      return KD_ERROR;
    }
    attUend -= inParam;
  } else {
    if (attUstart + inParam > attUend) {
      return KD_ERROR;
    }
    
    attUstart += inParam;
  }
  return KD_OK;
}

ktStatus CdirectPathVector::extractTo(double inParam)
{
  if (CkwsDirectPath::extractTo(inParam) != KD_OK) {
    return KD_ERROR;
  }

  if (attReverse) {
    if (inParam > attUend - attUstart) {
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

ktStatus CdirectPathVector::reverse()
{
  if (CkwsDirectPath::reverse() != KD_OK) {
    return KD_ERROR;
  }
  attReverse = !attReverse;
  return KD_OK;
}
