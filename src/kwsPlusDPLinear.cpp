/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Eiichi Yoshida (AIST/LAAS-CNRS)

*/

/*! \addtogroup hpp
 *@{
 */

/*****************************************
INCLUDES
*******************************************/

#include "kwsPlusDPLinear.h"
#include "KineoWorks2/kwsDevice.h" // without this causes error when calling functions of kwsDevice
// #include "kwsioInterface.h"
#include <math.h>
#include <fstream>

CkwsPlusDPLinear::~CkwsPlusDPLinear()
{
}

// =========================================================================================

CkwsPlusDPLinearShPtr CkwsPlusDPLinear::create(const CkwsConfig &inStartCfg,
					       const CkwsConfig &inEndCfg,
					       const std::vector<double> &i_ratio_vector,
					       const CkwsSteeringMethodShPtr &inSteeringMethod)
{

  CkwsPlusDPLinear* pathPtr = new CkwsPlusDPLinear(inStartCfg, inEndCfg, i_ratio_vector, 
						   inSteeringMethod);
  CkwsPlusDPLinearShPtr pathShPtr(pathPtr);
  CkwsPlusDPLinearWkPtr pathWkPtr(pathShPtr) ;

  // Init should be at the end since CkwsDirectPath::init() calls computePrivateLength that is defined by
  // attArcLengthManager
  if (pathShPtr) {
    if (pathPtr->init(pathWkPtr) != KD_OK) {
      pathShPtr.reset()	;
    }
  }

  if (!pathShPtr)
  {
    //cerr << " \\\\ CkwsPlusDPLinear::create failed //// " << endl ;
    //cerr << "init : " << inStartCfg << endl;
    //cerr << "end : "  << inEndCfg << endl ;
  }
  else
  {
    // debug
    // cout << "================= CkwsPlusDPLinear::create succeeded ===============================" << endl;
    // cout << "init : " << inStartCfg << endl;
    // cout << "end : "  << inEndCfg << endl ;
    // -----------
  }

  return pathShPtr ;
}

// =========================================================================================

CkwsPlusDPLinearShPtr CkwsPlusDPLinear::createCopy(const CkwsPlusDPLinearConstShPtr &i_kwsPlusDPLinear)
{

  if(i_kwsPlusDPLinear != NULL)
  {
    CkwsPlusDPLinear* pathPtr = new CkwsPlusDPLinear(*i_kwsPlusDPLinear) ;
    CkwsPlusDPLinearShPtr pathShPtr(pathPtr) ;
    CkwsPlusDPLinearWkPtr pathWkPtr(pathShPtr) ;

    if(pathPtr->init(pathWkPtr) != KD_OK)
    {
      pathShPtr.reset() ;
      return pathShPtr;
    }

    return pathShPtr;

  }
  else return CkwsPlusDPLinearShPtr() ;

}

// =========================================================================================

CkwsAbstractPathShPtr CkwsPlusDPLinear::clone() const
{

  return CkwsPlusDPLinear::createCopy(m_weakPtr.lock());

}

/*****************************************
 PROTECTED METHODS
*******************************************/


// =========================================================================================

CkwsPlusDPLinear::CkwsPlusDPLinear(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg,
				   const std::vector<double> &i_ratio_vector,
				   const CkwsSteeringMethodShPtr &inSteeringMethod) :
  CkwsDPLinear(inStartCfg, inEndCfg, inSteeringMethod), m_ratio_vector(i_ratio_vector)
{

}

CkwsPlusDPLinear::CkwsPlusDPLinear(const CkwsPlusDPLinear &inDirectPath) :
 CkwsDPLinear(inDirectPath)
{
}

// =========================================================================================

ktStatus CkwsPlusDPLinear::init(const CkwsPlusDPLinearWkPtr &inWeakPtr)
{

  ktStatus success = CkwsDirectPath::init(inWeakPtr) ;

  if (KD_OK == success) m_weakPtr = inWeakPtr;

  return success ;

}

// =========================================================================================

double CkwsPlusDPLinear::computePrivateLength() const
{
  
  return CkwsDPLinear::computePrivateLength();
}

// =========================================================================================

void CkwsPlusDPLinear::interpolate(double i_s, CkwsConfig& o_cfg) const
{
  
  CkwsDPLinear::interpolate(i_s, o_cfg);
}

// =========================================================================================

void CkwsPlusDPLinear::maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double> & outVectorDeriv) const
{
  KWS_PRECONDITION( m_start.size() == device()->countDofs() );

  CkwsDeviceShPtr dev(device());
  outVectorDeriv.resize(dev->countDofs()); // that causes error with 2.04

  // For each DOF, the derivative value will be multiplied by the corresponding ratio.
  // if minus or zero, the value will be set to 1.

  // get the maxAbsoluteDerive from the parent class of Linear Steering Method
  std::vector<double> linearVectorDeriv(dev->countDofs());
  CkwsDPLinear::maxAbsoluteDerivative(inFrom, inTo, linearVectorDeriv);

  std::vector<double> ratio_vector(m_ratio_vector);
  if(outVectorDeriv.size() > ratio_vector.size()){
    ratio_vector.resize(outVectorDeriv.size(), 1.0);
  }

  for(unsigned int i=0; i<outVectorDeriv.size(); i++){
    if(ratio_vector[i] < 0)
      ratio_vector[i] = 1.0;
    outVectorDeriv[i] = linearVectorDeriv[i]*ratio_vector[i];
  }  
}

/** @}
 */
