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

#include "kwsPlusSMLinear.h"
#include "kwsPlusDPLinear.h"

/*****************************************
PUBLIC METHODS
*******************************************/

// =========================================================================================

CkwsPlusSMLinear::CkwsPlusSMLinear(const std::vector<double> &i_ratio_vector, 
				   bool i_oriented) 
  : m_oriented(i_oriented), m_ratio_vector(i_ratio_vector)
{
	//nothing to do
}

// =========================================================================================

CkwsPlusSMLinear::~CkwsPlusSMLinear() {
	//nothing to do
}

// =========================================================================================

ktStatus CkwsPlusSMLinear::init(const CkwsSteeringMethodWkPtr& i_smWkPtr) {
 
    ktStatus  success = CkwsSteeringMethod::init(i_smWkPtr);

    if(KD_OK == success) m_weakPtr = i_smWkPtr;
    
    return success;
}

// ==========================================================================================

CkwsPlusSMLinearShPtr CkwsPlusSMLinear::create(const std::vector<double> &i_ratio_vector,
					       bool i_oriented)
{
  CkwsPlusSMLinear*  flatPtr = new CkwsPlusSMLinear(i_ratio_vector, i_oriented);
  CkwsPlusSMLinearShPtr flatShPtr(flatPtr);
  CkwsPlusSMLinearWkPtr flatWkPtr(flatShPtr);

  cout << "steering method create" << endl;

  if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();

  return flatShPtr;
}



// =========================================================================================

CkwsDirectPathShPtr CkwsPlusSMLinear::makeDirectPath (const CkwsConfig &i_startCfg, const CkwsConfig &i_endCfg) {
	
  // bool oriented = isOriented();
  return CkwsPlusDPLinear::create(i_startCfg, i_endCfg, m_ratio_vector, m_weakPtr.lock()) ;
 
}

// ==========================================================================================

bool CkwsPlusSMLinear::isOriented() const {
  return m_oriented ;
}



/** @}
 */
