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

/* The following macro registers the class with the class registry, assigns the static CLASS attribute and implements the classObject() virtual method */

KIT_DEFINE_CLASS( CflicSteeringMethod );

// =========================================================================================

CflicSteeringMethod::CflicSteeringMethod(bool i_oriented) : m_oriented(i_oriented) {
	//nothing to do
}

// ============================================================================
CflicSteeringMethod::CflicSteeringMethod
(const CflicSteeringMethod& steeringMethod) :
  CkwsSteeringMethod (steeringMethod),
  m_oriented (steeringMethod.m_oriented)
{
}

// =========================================================================================

CflicSteeringMethod::CflicSteeringMethod()
{
	//nothing to do
}

// =========================================================================================

CflicSteeringMethod::~CflicSteeringMethod() {
	//nothing to do
}

// =========================================================================================

ktStatus CflicSteeringMethod::init(const CflicSteeringMethodWkPtr& i_smWkPtr) {
 
    ktStatus  success = CkwsSteeringMethod::init(i_smWkPtr);

    if(KD_OK == success) m_weakPtr = i_smWkPtr;
    
    return success;
}

// ===========================================================================

CflicSteeringMethodShPtr CflicSteeringMethod::create(bool i_oriented)
{
	CflicSteeringMethod*  flatPtr = new CflicSteeringMethod(i_oriented);
	CflicSteeringMethodShPtr flatShPtr(flatPtr);
	CflicSteeringMethodWkPtr flatWkPtr(flatShPtr);

	ODEBUG2("steering method create.");

	if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();

	return flatShPtr;
}

// ===========================================================================

CflicSteeringMethodShPtr CflicSteeringMethod::createCopy
(const CflicSteeringMethodConstShPtr& steeringMethod)
{
  CflicSteeringMethod*  flatPtr = new CflicSteeringMethod(*steeringMethod);
  CflicSteeringMethodShPtr flatShPtr(flatPtr);
  CflicSteeringMethodWkPtr flatWkPtr(flatShPtr);

  ODEBUG2("steering method create.");
  
  if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();
  
  return flatShPtr;
}

// ===========================================================================

void CflicSteeringMethod::encodeWithCoder(const CkitCoderShPtr& i_coder) const
{
  // Always call parent class coding:
  CkwsSteeringMethod::encodeWithCoder(i_coder);
	
  i_coder->encodeBool(m_oriented,"oriented");
}

// ===========================================================================

ktStatus CflicSteeringMethod::initWithCoder(const CkitCoderShPtr& i_coder,
					    const CkitCodableShPtr& i_self)
{
  ktStatus success = CkwsSteeringMethod::initWithCoder(i_coder, i_self);
  if(KD_OK == success)
    {
      m_weakPtr = KIT_DYNAMIC_PTR_CAST(CflicSteeringMethod,i_self);
      m_oriented = i_coder->decodeBool("oriented");
    }
  return success;
}

// ===========================================================================

CkwsSteeringMethodShPtr CflicSteeringMethod::clone () const
{
  return CflicSteeringMethod::createCopy (this->m_weakPtr.lock ());
}

// ============================================================================

CkwsDirectPathShPtr CflicSteeringMethod::makeDirectPath
(const CkwsConfig &i_startCfg, const CkwsConfig &i_endCfg) const
{
  bool oriented = isOriented (i_startCfg.configSpace ());
  return CflicDirectPath::create(i_startCfg, i_endCfg, m_weakPtr.lock(),
				 oriented) ;
 
}

// ==========================================================================================

bool CflicSteeringMethod::isOriented (const CkwsConfigSpaceConstShPtr &) const
{
  return m_oriented ;
}

bool CflicSteeringMethod::isOriented () const
{
  return m_oriented ;
}

/** @}
 */
