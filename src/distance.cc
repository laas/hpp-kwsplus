/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Sebastian Dalibar
           and Alireza NAKHAEI

*/

/*****************************************
INCLUDES
*******************************************/

#include <hpp/kwsplus/direct-path/flic-steering-method.hh>
#include <hpp/kwsplus/direct-path/flic-direct-path.hh>
#include <hpp/kwsplus/direct-path/distance.hh>

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CkwsPlusDistance:" << x << std::endl
#define ODEBUG2(x) std::cout << "CkwsPlusDistance:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusDistance:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CkwsPlusDistance:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusDistance:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CkwsPlusDistance:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
PUBLIC METHODS
*******************************************/

// =========================================================================================

CkwsPlusDistance::CkwsPlusDistance(CkwsSteeringMethodShPtr inStreeingMethod)

{
	attSteeringMethod= inStreeingMethod;
	//nothing to do
}

// =========================================================================================

CkwsPlusDistance::~CkwsPlusDistance()
{
	//nothing to do
}


// =========================================================================================

ktStatus CkwsPlusDistance::init ( const CkwsPlusDistanceWkPtr& i_smWkPtr )
{

	ktStatus  success = CkwsMetric::init ( i_smWkPtr );

	if ( KD_OK == success ) m_weakPtr = i_smWkPtr;

	return success;
}

// ==========================================================================================

CkwsPlusDistanceShPtr CkwsPlusDistance:: create ( CkwsSteeringMethodShPtr inStreeingMethod )
{
	CkwsPlusDistance*  flatPtr = new CkwsPlusDistance(inStreeingMethod);
	CkwsPlusDistanceShPtr flatShPtr ( flatPtr );
	CkwsPlusDistanceWkPtr flatWkPtr ( flatShPtr );

	ODEBUG2( "CkwsPlusDistance create" );

	if ( flatPtr->init ( flatWkPtr ) != KD_OK ) flatShPtr.reset();

	return flatShPtr;
}

// ==========================================================================================

CkwsPlusDistanceShPtr CkwsPlusDistance::createCopy ( const CkwsPlusDistanceShPtr& inDistance )
{
	CkwsPlusDistance*  flatPtr = new CkwsPlusDistance(*inDistance);
	CkwsPlusDistanceShPtr flatShPtr ( flatPtr );
	CkwsPlusDistanceWkPtr flatWkPtr ( flatShPtr );

	ODEBUG2( "CkwsPlusDistance create copy" );

	if ( flatPtr->init ( flatWkPtr ) != KD_OK )
	  {
	    flatShPtr.reset();
	    return flatShPtr;
	  }

	if (inDistance->attSteeringMethod)
	  flatShPtr->attSteeringMethod = inDistance->attSteeringMethod;
	else
	  {
	    ODEBUG1(":createCopy: attSteeringMethod does not exist in the copied distance.");
	    return CkwsPlusDistanceShPtr ();
	  }

	return flatShPtr;
}

// ==========================================================================================

CkwsMetricShPtr CkwsPlusDistance::clone () const
{
  return createCopy (m_weakPtr.lock ());
}

// ==========================================================================================

double CkwsPlusDistance::distance ( const CkwsConfig &i_cfg1, const CkwsConfig &i_cfg2 ) const
{
	CkwsDirectPathShPtr  flicDirectPath = attSteeringMethod->makeDirectPath ( i_cfg1 ,i_cfg2 );
	if ( !flicDirectPath )
	{
		return HUGE_VAL;
	}
	return flicDirectPath->length();
}

double CkwsPlusDistance::distanceForSorting (const CkwsConfig& cfg1, const CkwsConfig& cfg2) const
{
  return distance (cfg1, cfg2);
}
