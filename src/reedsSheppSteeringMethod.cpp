/*
            Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
            Implemented by  Mathieu Poirier (LAAS-CNRS)
 
*/

/*****************************************
            INCLUDES
*******************************************/

#include "reedsSheppSteeringMethod.h"
#include "reedsSheppDirectPath.h"

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CreedsSheppSteeringMethod:" << x << std::endl
#define ODEBUG2(x) std::cout << "CreedsSheppSteeringMethod:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CreedsSheppSteeringMethod:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CreedsSheppSteeringMethod:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CreedsSheppSteeringMethod:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CreedsSheppSteeringMethod:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
            PUBLIC METHODS
*******************************************/

// =========================================================================================

CreedsSheppSteeringMethod::CreedsSheppSteeringMethod(bool i_oriented, double inRadius ) : m_oriented(i_oriented)
{
        attRadius = inRadius ;
	attType   = RS_ALL ;
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

CreedsSheppSteeringMethodShPtr CreedsSheppSteeringMethod::create( double inRadius, bool i_oriented)
{
  CreedsSheppSteeringMethod*  RSPtr = new CreedsSheppSteeringMethod(i_oriented, inRadius);
        CreedsSheppSteeringMethodShPtr RSShPtr(RSPtr);
        CreedsSheppSteeringMethodWkPtr RSWkPtr(RSShPtr);

        ODEBUG2( "steering method create" );

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
