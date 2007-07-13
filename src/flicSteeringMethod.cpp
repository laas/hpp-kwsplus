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

#include "flicSteeringMethod.h"
#include "flicDirectPath.h"


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

	cout << "steering method create" << endl;

	if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();

	return flatShPtr;
}



// =========================================================================================

CkwsDirectPathShPtr CflicSteeringMethod::makeDirectPath (const CkwsConfig &i_startCfg, const CkwsConfig &i_endCfg) {
	
  bool oriented = isOriented();
  return CflicDirectPath::create(i_startCfg, i_endCfg, m_weakPtr.lock(), oriented) ;
 
}

// ==========================================================================================

bool CflicSteeringMethod::isOriented() const {
  return m_oriented ;
}


/** @}
 */
