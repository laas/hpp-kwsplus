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

#include "kwsPlus/directPath/kwsPlusSMLinear.h"
#include "kwsPlus/directPath/kwsPlusDPLinear.h"

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CkwsPlusSMLinear:" << x << std::endl
#define ODEBUG2(x) std::cout << "CkwsPlusSMLinear:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusSMLinear:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CkwsPlusSMLinear:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusSMLinear:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CkwsPlusSMLinear:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
PUBLIC METHODS
*******************************************/

// =========================================================================================

CkwsPlusSMLinear::CkwsPlusSMLinear(const std::vector<double> &inRatioVector, 
				   bool inOriented) 
  : attOriented(inOriented), attRatioVector(inRatioVector)
{
	//nothing to do
}

// =========================================================================================

CkwsPlusSMLinear::~CkwsPlusSMLinear() {
	//nothing to do
}

// =========================================================================================

ktStatus CkwsPlusSMLinear::init(const CkwsSteeringMethodWkPtr& inSmWeakPtr) {
 
    ktStatus  success = CkwsSteeringMethod::init(inSmWeakPtr);

    if(KD_OK == success) attWeakPtr = inSmWeakPtr;
    
    return success;
}

// ==========================================================================================

CkwsPlusSMLinearShPtr CkwsPlusSMLinear::create(const std::vector<double> &inRatioVector,
					       bool inOriented)
{
  CkwsPlusSMLinear*  flatPtr = new CkwsPlusSMLinear(inRatioVector, inOriented);
  CkwsPlusSMLinearShPtr flatShPtr(flatPtr);
  CkwsPlusSMLinearWkPtr flatWkPtr(flatShPtr);

  ODEBUG2( "steering method create" );

  if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();

  return flatShPtr;
}



// =========================================================================================

CkwsDirectPathShPtr CkwsPlusSMLinear::makeDirectPath (const CkwsConfig &inStartConfig, const CkwsConfig &inEndConfig) {
	
  // bool oriented = isOriented();
  return CkwsPlusDPLinear::create(inStartConfig, inEndConfig, attRatioVector, attWeakPtr.lock()) ;
 
}

// ==========================================================================================

bool CkwsPlusSMLinear::isOriented() const {
  return attOriented ;
}



/** @}
 */
