/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Eiichi Yoshida (AIST/LAAS-CNRS)

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
    const CkwsSteeringMethodShPtr &inSteeringMethod)
{

  CkwsPlusDPLinear* pathPtr = new CkwsPlusDPLinear(inStartCfg, inEndCfg,inSteeringMethod);
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
                                 const CkwsSteeringMethodShPtr &inSteeringMethod) :
    CkwsDPLinear(inStartCfg, inEndCfg, inSteeringMethod)
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

void CkwsPlusDPLinear::maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double> & outVectorDeriv) const
{

}

