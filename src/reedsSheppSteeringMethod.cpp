/*
            Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
            Implemented by  Mathieu Poirier (LAAS-CNRS)
 
*/

/*****************************************
            INCLUDES
*******************************************/

#include "kwsPlus/directPath/reedsSheppSteeringMethod.h"
#include "kwsPlus/directPath/reedsSheppDirectPath.h"

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

// The following macro registers the class with the class registry,
// assigns the static CLASS attribute
// and implements the classObject() virtual method.
KIT_DEFINE_CLASS( CreedsSheppSteeringMethod );

// =========================================================================================

CreedsSheppSteeringMethod::CreedsSheppSteeringMethod(bool i_oriented,
						     double inRadius,
						     ERsCurveType type)
  : attOriented(i_oriented), attRadius (inRadius), attType (type)
{
}

// =========================================================================================

CreedsSheppSteeringMethod::CreedsSheppSteeringMethod(const CreedsSheppSteeringMethod& i_steeringMethod)
	: attOriented(i_steeringMethod.isOriented()),attRadius(i_steeringMethod.radius()),attType(i_steeringMethod.type())
{
	//nothing to do
}

// =========================================================================================

CreedsSheppSteeringMethod::CreedsSheppSteeringMethod() 
	: attOriented(false),attRadius(0.),attType(RS_ALL)
{
	//nothing to do
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

CreedsSheppSteeringMethodShPtr CreedsSheppSteeringMethod::create(double inRadius,
								 bool i_oriented,
								 ERsCurveType type)
{
  CreedsSheppSteeringMethod*  RSPtr = new CreedsSheppSteeringMethod(i_oriented, inRadius, type);
        CreedsSheppSteeringMethodShPtr RSShPtr(RSPtr);
        CreedsSheppSteeringMethodWkPtr RSWkPtr(RSShPtr);

        ODEBUG2( "steering method create" );

        if(RSPtr->init(RSWkPtr) != KD_OK)
                RSShPtr.reset();

        return RSShPtr;
}

// ==========================================================================================

CreedsSheppSteeringMethodShPtr CreedsSheppSteeringMethod::createCopy(const CreedsSheppSteeringMethodConstShPtr& i_steeringMethod)
{
  CreedsSheppSteeringMethod*  RSPtr = new CreedsSheppSteeringMethod(*i_steeringMethod);
        CreedsSheppSteeringMethodShPtr RSShPtr(RSPtr);

        ODEBUG2( "steering method create" );

        if(RSPtr->init(RSShPtr) != KD_OK)
                RSShPtr.reset();

        return RSShPtr;
}

// =========================================================================================

CkwsDirectPathShPtr CreedsSheppSteeringMethod::makeDirectPath (const CkwsConfig &i_startCfg, const CkwsConfig &i_endCfg) const
{

        //    bool oriented = isOriented();
  return CreedsSheppDirectPath::create(i_startCfg, i_endCfg, m_weakPtr.lock(), attRadius, attType) ;

}

// ==========================================================================================

bool CreedsSheppSteeringMethod::isOriented() const
{
        return attOriented ;
}

// ==========================================================================================

double CreedsSheppSteeringMethod::radius()const
{
	return attRadius;
}

// ==========================================================================================

ERsCurveType CreedsSheppSteeringMethod::type()const
{
	return attType;
}

// ==========================================================================================

CkwsSteeringMethodShPtr CreedsSheppSteeringMethod::clone() const
{
	return CreedsSheppSteeringMethod::createCopy(KIT_DYNAMIC_PTR_CAST(const CreedsSheppSteeringMethod, m_weakPtr.lock()));
}

// ==========================================================================================

void CreedsSheppSteeringMethod::encodeWithCoder(const CkitCoderShPtr& i_coder) const
{
  // Always call parent class coding:
  CkwsSteeringMethod::encodeWithCoder(i_coder);
  
  // Encode object attributes.
  // By convention, use identifier strings that match
  // attribute names.
  i_coder->encodeDouble(attRadius, "attRadius");
  i_coder->encodeInt(attType, "attType");
  i_coder->encodeBool(attOriented, "oriented");
}

// ==========================================================================================

ktStatus CreedsSheppSteeringMethod::initWithCoder(const CkitCoderShPtr& i_coder, const CkitCodableShPtr& i_self)
{
  ktStatus success = CkwsSteeringMethod::initWithCoder(i_coder, i_self);
  if(KD_OK == success)
  {
    m_weakPtr = KIT_DYNAMIC_PTR_CAST( CreedsSheppSteeringMethod, i_self );
    attRadius = i_coder->decodeDouble("attRadius");
    attType = (ERsCurveType)i_coder->decodeInt("attType");
    attOriented = i_coder->decodeBool("oriented");
  }
  return success;
}
