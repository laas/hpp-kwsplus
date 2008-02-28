/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Sebastian Dalibar
           and Alireza NAKHAEI

*/

/*! \addtogroup hpp
 *@{
 */

/*****************************************
INCLUDES
*******************************************/

#include "flicSteeringMethod.h"
#include "flicDirectPath.h"
#include "kwsPlusDistance.h"


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

	ktStatus  success = CkwsDistance::init ( i_smWkPtr );

	if ( KD_OK == success ) m_weakPtr = i_smWkPtr;

	return success;
}

// ==========================================================================================

CkwsPlusDistanceShPtr CkwsPlusDistance:: create ( CkwsSteeringMethodShPtr inStreeingMethod )
{
	CkwsPlusDistance*  flatPtr = new CkwsPlusDistance(inStreeingMethod);
	CkwsPlusDistanceShPtr flatShPtr ( flatPtr );
	CkwsPlusDistanceWkPtr flatWkPtr ( flatShPtr );

	cout << "CkwsPlusDistance create" << endl;

	if ( flatPtr->init ( flatWkPtr ) != KD_OK ) flatShPtr.reset();

	return flatShPtr;
}

// ==========================================================================================

double CkwsPlusDistance::distance ( const CkwsConfig &i_cfg1, const CkwsConfig &i_cfg2 ) const
{
	CkwsDirectPathShPtr  flicDirectPath = attSteeringMethod->makeDirectPath ( i_cfg1 ,i_cfg2 );
	if ( !flicDirectPath )
	{
		return 1000000;
	}
	return flicDirectPath->length();
}


/** @}
 */
