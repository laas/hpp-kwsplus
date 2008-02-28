/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Sebastian Dalibar
           and Alireza NAKHAEI
*/



#ifndef KWS_PLUS_DISTANCE_H
#define KWS_PLUS_DISTANCE_H

/*************************************
INCLUDE
**************************************/

#include <iostream>

#include "KineoWorks2/kwsSteeringMethod.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDistance.h"



using namespace std ;

/**

\addtogroup hpp
@{

*/

/*************************************
STRUCTURES
**************************************/

KIT_PREDEF_CLASS ( CkwsPlusDistance );

/*************************************
CLASS
**************************************/
/**
   \brief Implement a distance which enable the user to measure distance based on a specific steering method.

   CkwsPlusDistance is herited from kwsDistance and eable users to use it as a steering method base distance calculator .

*/
class CkwsPlusDistance : public CkwsDistance
{


	public :
		/**
		   \brief   Destructor.
		*/
		virtual ~CkwsPlusDistance() ;

		/**
		   \brief  create a steering method and return the shared pointer corresponding
		   \param  is_oriented : a bool (default = true) ;
		   \return a shared pointer on the Steering Method
		*/
		static CkwsPlusDistanceShPtr create(CkwsSteeringMethodShPtr inStreeingMethod) ;

		ktStatus init ( const CkwsPlusDistanceWkPtr& i_smWkPtr );

		virtual double distance ( const CkwsConfig &i_cfg1, const CkwsConfig &i_cfg2 ) const;

	protected:


	private:


		CkwsPlusDistance(CkwsSteeringMethodShPtr inStreeingMethod);

		CkwsSteeringMethodShPtr attSteeringMethod;

		CkwsPlusDistanceWkPtr m_weakPtr;
} ;


/**
   @}
*/
#endif

