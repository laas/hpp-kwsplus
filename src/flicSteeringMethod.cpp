/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux
           and Mathieu Poirier (LAAS-CNRS)

*/

/*! \addtogroup hpp
 *@{
 */

/*****************************************
INCLUDES
*******************************************/

#include "kwsPlus/directPath/flicSteeringMethod.h"
#include "kwsPlus/directPath/flicDirectPath.h"

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CflicSteeringMethod:" << x << std::endl
#define ODEBUG2(x) std::cout << "CflicSteeringMethod:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CflicSteeringMethod:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CflicSteeringMethod:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CflicSteeringMethod:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CflicSteeringMethod:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
PUBLIC METHODS
*******************************************/

// =========================================================================================

CflicSteeringMethod::CflicSteeringMethod(bool i_oriented) : m_oriented(i_oriented) {
	//nothing to do
}

// =========================================================================================

CflicSteeringMethod::~CflicSteeringMethod() {
	//nothing to do
}

// =========================================================================================

ktStatus CflicSteeringMethod::init(const CkwsSteeringMethodWkPtr& i_smWkPtr) {
 
    ktStatus  success = CkwsSteeringMethod::init(i_smWkPtr);

    if(KD_OK == success) m_weakPtr = i_smWkPtr;
    
    return success;
}

// ==========================================================================================

CflicSteeringMethodShPtr CflicSteeringMethod::create(bool i_oriented)
{
	CflicSteeringMethod*  flatPtr = new CflicSteeringMethod(i_oriented);
	CflicSteeringMethodShPtr flatShPtr(flatPtr);
	CflicSteeringMethodWkPtr flatWkPtr(flatShPtr);

	ODEBUG2("steering method create.");

	if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();

	return flatShPtr;
}



// =========================================================================================

CkwsDirectPathShPtr
CflicSteeringMethod::makeDirectPath (const CkwsConfig &i_startCfg,
				     const CkwsConfig &i_endCfg) const
{
	
  bool oriented = isOriented();
  return CflicDirectPath::create(i_startCfg, i_endCfg, m_weakPtr.lock(), oriented) ;
 
}

// ==========================================================================================

bool CflicSteeringMethod::isOriented() const {
  return m_oriented ;
}


/** @}
 */
