/*
            Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
            Implemented by  Mathieu Poirier (LAAS-CNRS)
 
*/

/*! \addtogroup hpp
 *@{
 */

/*****************************************
            INCLUDES
*******************************************/

#include "reedsSheppSteeringMethod.h"
#include "reedsSheppDirectPath.h"


/*****************************************
            PUBLIC METHODS
*******************************************/

// =========================================================================================

CreedsSheppSteeringMethod::CreedsSheppSteeringMethod(bool i_oriented, double inRadius, ERsCurveType inType ) : m_oriented(i_oriented)
{
        attRadius = inRadius ;
	attType   = inType ;
}

// =========================================================================================

CreedsSheppSteeringMethod::~CreedsSheppSteeringMethod()
{
        //nothing to do
}

// =========================================================================================

ktStatus CreedsSheppSteeringMethod::init(const CkwsSteeringMethodWkPtr& i_smWkPtr)
{

        ktStatus  success = CkwsSteeringMethod::init(i_smWkPtr);

        if(KD_OK == success)
                m_weakPtr = i_smWkPtr;

        return success;
}

// ==========================================================================================

CreedsSheppSteeringMethodShPtr CreedsSheppSteeringMethod::create( double inRadius, ERsCurveType inType, bool i_oriented)
{
  CreedsSheppSteeringMethod*  RSPtr = new CreedsSheppSteeringMethod(i_oriented, inRadius, inType);
        CreedsSheppSteeringMethodShPtr RSShPtr(RSPtr);
        CreedsSheppSteeringMethodWkPtr RSWkPtr(RSShPtr);

        cout << "steering method create" << endl;

        if(RSPtr->init(RSWkPtr) != KD_OK)
                RSShPtr.reset();

        return RSShPtr;
}



// =========================================================================================

CkwsDirectPathShPtr CreedsSheppSteeringMethod::makeDirectPath (const CkwsConfig &i_startCfg, const CkwsConfig &i_endCfg)
{

        //    bool oriented = isOriented();
  return CreedsSheppDirectPath::create(i_startCfg, i_endCfg, m_weakPtr.lock(), attRadius, attType) ;

}

// ==========================================================================================

bool CreedsSheppSteeringMethod::isOriented() const
{
        return m_oriented ;
}


/** @}
 */
