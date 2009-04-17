/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Eiichi Yoshida (AIST/LAAS-CNRS)

*/

/*****************************************
INCLUDES
*******************************************/

#include "kwsPlus/directPath/kwsPlusDPLinear.h"
#include "KineoWorks2/kwsDevice.h" // without this causes error when calling functions of kwsDevice
// #include "kwsioInterface.h"
#include <math.h>
#include <fstream>

// Select lecel of verbosity at configuration.
#if DEBUG==2
#define ODEBUG2(x) std::cout << "CkwsPlusDPLinear:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusDPLinear:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CkwsPlusDPLinear:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

CkwsPlusDPLinear::~CkwsPlusDPLinear()
{
}

// =========================================================================================

CkwsPlusDPLinearShPtr CkwsPlusDPLinear::create(const CkwsConfig &inStartCfg,
					       const CkwsConfig &inEndCfg,
					       const std::vector<double> &inRatioVector,
					       const CkwsSteeringMethodShPtr &inSteeringMethod)
{

  CkwsPlusDPLinear* pathPtr = new CkwsPlusDPLinear(inStartCfg, inEndCfg, inRatioVector, 
						   inSteeringMethod);
  CkwsPlusDPLinearShPtr pathShPtr(pathPtr);
  CkwsPlusDPLinearWkPtr pathWkPtr(pathShPtr) ;

  if (pathShPtr) {
    if (pathPtr->init(pathWkPtr) != KD_OK) {
      pathShPtr.reset()	;
    }
  }

  return pathShPtr ;
}

// =========================================================================================

CkwsPlusDPLinearShPtr CkwsPlusDPLinear::createCopy(const CkwsPlusDPLinearConstShPtr &inKwsPlusDPLinear)
{
  if(inKwsPlusDPLinear) {
    CkwsPlusDPLinear* pathPtr = new CkwsPlusDPLinear(*inKwsPlusDPLinear) ;
    CkwsPlusDPLinearShPtr pathShPtr(pathPtr) ;
    CkwsPlusDPLinearWkPtr pathWkPtr(pathShPtr) ;

    if(pathPtr->init(pathWkPtr) != KD_OK) {
      pathShPtr.reset() ;
    }

    return pathShPtr;

  }
  else return CkwsPlusDPLinearShPtr() ;

}

// =========================================================================================

CkwsAbstractPathShPtr CkwsPlusDPLinear::clone() const
{

  return CkwsPlusDPLinear::createCopy(attWeakPtr.lock());

}

/*****************************************
 PROTECTED METHODS
*******************************************/


// =========================================================================================

CkwsPlusDPLinear::CkwsPlusDPLinear(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg,
				   const std::vector<double> &inRatioVector,
				   const CkwsSteeringMethodShPtr &inSteeringMethod) :
  CkwsPlusDirectPath(inStartCfg, inEndCfg, inSteeringMethod), 
  attRatioVector(inRatioVector)
{
  attLinearDP = CkwsDPLinear::create(inStartCfg, inEndCfg, inSteeringMethod);
}

CkwsPlusDPLinear::CkwsPlusDPLinear(const CkwsPlusDPLinear &inDirectPath) :
  CkwsPlusDirectPath(inDirectPath),
  attRatioVector(inDirectPath.attRatioVector)
{
  attLinearDP = CkwsDPLinear::createCopy(inDirectPath.attLinearDP);
}

// =========================================================================================

ktStatus CkwsPlusDPLinear::init(const CkwsPlusDPLinearWkPtr &inWeakPtr)
{
  if (!attLinearDP) {
    return KD_ERROR;
  }

  if (CkwsPlusDirectPath::init(inWeakPtr) != KD_OK) {
    return KD_ERROR;
  }

  attWeakPtr = inWeakPtr;
  return KD_OK;

}

// =========================================================================================

double CkwsPlusDPLinear::computePrivateLength() const
{
  
  return attLinearDP->length();
}

// =========================================================================================

void CkwsPlusDPLinear::interpolate(double inParam, CkwsConfig& outConfig) const
{
  attLinearDP->getConfigAtDistance(inParam, outConfig);
}

// =========================================================================================

void CkwsPlusDPLinear::maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double> & outVectorDeriv) const
{
  KWS_PRECONDITION( privateStart().size() == device()->countDofs() );

  CkwsDeviceShPtr dev(device());
  outVectorDeriv.resize(dev->countDofs()); // that causes error with 2.04

  // For each DOF, the derivative value will be multiplied by the corresponding ratio.
  // if minus or zero, the value will be set to 1.

  // get the maxAbsoluteDerive from the parent class of Linear Steering Method
  std::vector<double> linearVectorDeriv(dev->countDofs());

  /*
    As CkwsDPLinear::maxAbsoluteDerivative is protected and 
    CkwsDirectPath::maxAbsoluteDerivative is public, we need to cast attLinearDP
    into CkwsDirectPath to call this method.
  */
  CkwsDirectPathShPtr kwsDirectPath = attLinearDP;
  kwsDirectPath->maxAbsoluteDerivative(inFrom, inTo, linearVectorDeriv);

  std::vector<double> ratio_vector(attRatioVector);
  if(outVectorDeriv.size() > ratio_vector.size()){
    ratio_vector.resize(outVectorDeriv.size(), 1.0);
  }

  for(unsigned int i=0; i<outVectorDeriv.size(); i++){
    if(ratio_vector[i] < 0)
      ratio_vector[i] = 1.0;
    outVectorDeriv[i] = linearVectorDeriv[i]*ratio_vector[i];
  }  
}

ktStatus 
CkwsPlusDPLinear::getVelocityAtDistanceAtConstruction(double inDistance, 
						      std::vector<double>& outVelocity) const
{
  CkwsConfig startConfig = privateStart();
  CkwsConfig endConfig = privateEnd();
  double l = privateLength();

  if (outVelocity.size() != startConfig.device()->countDofs()) {
    ODEBUG1(":getVelocityAtDistanceAtConstruction: vector size = "
	    << outVelocity.size() 
	    << " and configuration dimension = "
	    << startConfig.device()->countDofs()
	    << " do not fit.");
    return KD_ERROR;
  }
  CkwsConfig middleConfig(startConfig);
  attLinearDP->getConfigAtParam (.5*l, middleConfig);

  for (unsigned int iDof=0; iDof < outVelocity.size(); iDof++) {
    outVelocity[iDof] = 2*(middleConfig.dofValue(iDof)-
			   startConfig.dofValue(iDof))/(l);
  }
  return KD_OK;
}
