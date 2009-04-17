/*
  Copyright 2007 Florent Lamiraux
*/

#include <iostream>
#include "kwsPlus/directPath/directPathVector.h"
#include "kwsioInterface.h"

// #define DEBUG 2
#if DEBUG==2
#define ODEBUG2(x) std::cout << "CdirectPathVector:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CdirectPathVector:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CdirectPathVector:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

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
  ODEBUG2("CdirectPathVector called.");

  /*
  CdirectPathVectorShPtr dp; 
  std::vector<CkwsDirectPathConstShPtr> dpVector;

  for(unsigned int i = attRankStart; i <= attRankEnd; i++){
    dpVector.push_back(attDirectPathVector[i]);
  }
  CkwsConfig cfg_start(device()), cfg_end(device());
  dpVector[0]->getConfigAtStart(cfg_start);
  dpVector[dpVector.size()-1]->getConfigAtEnd(cfg_end);
  dp = CdirectPathVector::create(cfg_start, cfg_end, steeringMethod(), dpVector);
  dp->extractFrom(attStartLocal);
  dp->extractTo(length()-attEndLocal);
  if(isReversed())
    dp->reverse();
    
  return dp;
  */

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
  CkwsPlusDirectPath(inStartConfig, inEndConfig, inSteeringMethod), 
  attDirectPathVector(inDirectPathVector)

{
  attLength = 0.;
  //Compute length
  for (unsigned int iDP=0; iDP < inDirectPathVector.size(); iDP++) {
    if (inDirectPathVector[iDP]) {
      attLength += inDirectPathVector[iDP]->length();
    }
  }
  attRankStart = 0;
  attRankEnd = inDirectPathVector.size()-1;

  attStartLocal = 0.;
  attEndLocal = attDirectPathVector[attRankEnd]->length();
}


CdirectPathVector::CdirectPathVector(const CdirectPathVector& inDirectPathVector) :
  CkwsPlusDirectPath(inDirectPathVector), 
  attDirectPathVector(inDirectPathVector.attDirectPathVector),
  attLength(inDirectPathVector.attLength),
  attRankStart(inDirectPathVector.attRankStart),
  attRankEnd(inDirectPathVector.attRankEnd),
  attStartLocal(inDirectPathVector.attStartLocal),
  attEndLocal(inDirectPathVector.attEndLocal)
{
  /*
  attDirectPathVector.clear();
  for(unsigned int i=0; i<inDirectPathVector.attDirectPathVector.size(); i++){
    attDirectPathVector.push_back(CkwsDirectPath::createCopy(inDirectPathVector.attDirectPathVector[i]));
  }
  */
}

ktStatus CdirectPathVector::init(const CdirectPathVectorWkPtr& inWeakPtr)
{
  ktStatus success = CkwsPlusDirectPath::init(inWeakPtr) ;

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
  unsigned int rank;
  double localLength;
  getRankAtLength(inLength, rank, localLength);

  CkwsDirectPathConstShPtr directPath = attDirectPathVector[rank];
  directPath->getConfigAtDistance(localLength, outConfig);

#if  DEBUG==2
  CkwsConfig startCfg(outConfig.device()), endCfg(outConfig.device());
  directPath->getConfigAtStart(startCfg);
  directPath->getConfigAtEnd(endCfg);
#endif
  ODEBUG2(std::endl<<"at "<<inLength<<", rank "<<rank<<", localLength "<<localLength
	  <<" of "<<directPath->length() <<" total len "<<length()
	  <<std::endl<<" -- cfg"<<outConfig);
  ODEBUG2(" startCfg"<<startCfg<<std::endl<<"endCfg"<<endCfg<<std::endl);

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
  ODEBUG2(" in extractFrom(): old length "<<length());
  if (CkwsPlusDirectPath::extractFrom(inParam) != KD_OK) {
    ODEBUG1("ERROR : CkwsPlusDirectPath::extractFrom DID NOT WORK ");
    return KD_ERROR;
  }
  ODEBUG2(" new length "<<length());

  // uStart, uEnd, reversed of kwsPlusDirectPath should have been updated. 
  ktStatus status = KD_OK;
  double len;
  if(isReversed()){
    len = uEnd() < 0.  ? 0. : uEnd();
    status = getRankAtLength(len, attRankEnd, attEndLocal);
  }
  else{
    // get the rank and local length
    len = uStart() > privateLength() ? privateLength() : uStart();
    status = getRankAtLength(len, attRankStart, attStartLocal);
  }
  if(status)
    ODEBUG1(" ERROR : in extractFrom():getRankAtLength did not work ");

  return status;
}

ktStatus CdirectPathVector::extractTo(double inParam)
{
  ODEBUG2(" in extractTo(): old length "<<length());
  if (CkwsPlusDirectPath::extractTo(inParam) != KD_OK) {
    ODEBUG1("ERROR : CkwsPlusDirectPath::extractFrom DID NOT WORK ");
    return KD_ERROR;
  }
  ODEBUG2(" new length "<<length());

  ktStatus status = KD_OK;
  double len;
  if(isReversed()){
    len = uStart() < 0.  ? 0. : uStart();
    status = getRankAtLength(len, attRankStart, attStartLocal);
  }
  else{
    // get the rank and local length
    len = uEnd() > privateLength() ? privateLength() : uEnd();
    status = getRankAtLength(len, attRankEnd, attEndLocal);
  }
  if(status)
    ODEBUG1(" ERROR : in extractEnd(): getRankAtLength did not work ");

  return status;
}


ktStatus CdirectPathVector::reverse()
{
  if (CkwsPlusDirectPath::reverse() != KD_OK) {
    ODEBUG1("ERROR - : CkwsDirectPath::reverse DID NOT WORK");
    return KD_ERROR;
  }
  return KD_OK;
}

ktStatus CdirectPathVector::getRankAtLength(double inLength, unsigned int &oRank, 
					    double &localLength) const
{
  oRank = 0;
  localLength = inLength;
  bool finished = false;

  while ((oRank < attDirectPathVector.size()-1) && !finished) {
    if (localLength > attDirectPathVector[oRank]->length()) {
      localLength -= attDirectPathVector[oRank]->length();
      oRank++;
    }
    else {
      finished = true;
    }
  }
  // 
  if (localLength > attDirectPathVector[oRank]->length()) {
    if (oRank != attDirectPathVector.size()-1) {
      std::cerr << "CdirectPathVector::getRankAtLength: this should not happen" << std::endl;
      return KD_ERROR;
    }
    localLength = attDirectPathVector[oRank]->length();
  }
  return KD_OK;
}
