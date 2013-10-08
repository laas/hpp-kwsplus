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
#include "KineoWorks2/kwsMetric.h"



/**
   \addtogroup kwsPlus_DP
   @{
*/

/*************************************
STRUCTURES
**************************************/

HPP_KIT_PREDEF_CLASS ( CkwsPlusDistance );

/*************************************
CLASS
**************************************/
/**
   \brief Implement a distance which enable the user to measure distance based on a specific steering method.

   CkwsPlusDistance is herited from kwsMetric and enable users to use it as a steering method base distance calculator .

*/
class CkwsPlusDistance : public CkwsMetric
{


	public :
		/**
		   \brief   Destructor.
		*/
		virtual ~CkwsPlusDistance() ;

		/**
		   \brief  create a distance function related to a steering method.

		   \param inSteeringMethod the distance between two config is the length of the path returned by this steering method.

		   \return a shared pointer on the distance function
		*/
		static CkwsPlusDistanceShPtr create(CkwsSteeringMethodShPtr inSteeringMethod) ;
		/// Create copy and return shared pointer to newly allocated distance. 
		static CkwsPlusDistanceShPtr createCopy (const CkwsPlusDistanceShPtr& inDisance) ;

		CkwsMetricShPtr clone () const;

		ktStatus init ( const CkwsPlusDistanceWkPtr& i_smWkPtr );

		virtual double distance ( const CkwsConfig &i_cfg1, const CkwsConfig &i_cfg2 ) const;
		
		/// Same as distance
		virtual double distanceForSorting(const CkwsConfig& cfg1, const CkwsConfig& cfg2) const;

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

