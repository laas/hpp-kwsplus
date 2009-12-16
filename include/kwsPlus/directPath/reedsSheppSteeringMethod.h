/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Implemented by Mathieu Poirier (LAAS-CNRS)   
*/


#ifndef __REEDS_SHEEP_STEERING_METHOD_H__
#define __REEDS_SHEEP_STEERING_METHOD_H__

/*************************************
            INCLUDE
**************************************/

#include <iostream>
#include "KineoWorks2/kwsSteeringMethod.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDefine.h"
#include "kwsPlus/directPath/reedsSheppDirectPath.h"



/**
   \addtogroup RS
   @{
*/
/*************************************
STRUCTURES
**************************************/

KIT_PREDEF_CLASS( CreedsSheppSteeringMethod );

/*************************************
CLASS
**************************************/
/**
   \brief Steering Method for Reeds & Shepp derived from CkwsSteeringMethod.
*/
class CreedsSheppSteeringMethod : public CkwsSteeringMethod
{


public :
        /**
                  \brief   Destructor.
         */
        virtual ~CreedsSheppSteeringMethod() ;

        /**
                  \brief  create a steering method and return the shared pointer corresponding
                  \param  is_oriented : a bool (default = true) ;
                  \param inRadius : the radius for R&S method
                  \return a shared pointer on the Steering Method
         */
	static CreedsSheppSteeringMethodShPtr create(double inRadius,
						     bool is_oriented = true,
						     ERsCurveType type = RS_ALL) ;

        /**
                  \brief  Factory method that creates a new direct path between two configurations.
                  \param i_startCfg : the start config
                  \param i_endCfg : the end config
                  \return a shared pointer on the direct path just create
         */
        virtual CkwsDirectPathShPtr makeDirectPath(const CkwsConfig &i_startCfg, const CkwsConfig &i_endCfg) ;

        /**
                  \brief  Returns whether the steering method produces oriented or non-oriented paths.
                  \return true if the steering method is oriened or false
         */
        virtual bool isOriented() const ;

protected:

        ///weak pointer to itself
        CkwsSteeringMethodWkPtr   m_weakPtr ;
        /// oriented or not ?
        bool m_oriented ;
        /// the radius for the Reeds&Shepp Method
        double attRadius ;
        /// the type of calculation for the Reeds&Shepp Method
        ERsCurveType attType ;

        /**
                  \brief Constructor.
                  \param i_oriented : if true, the steering method will produce oriented paths
                  \param inRadius : the radius for R&S method
         */
        CreedsSheppSteeringMethod(bool i_oriented,
				  double inRadius,
				  ERsCurveType type = RS_ALL);

        /**
                  \brief Initialization of the CkwsSMLinear object.
                  \param         i_smWkPtr : weak pointer to the object itself
                  \return        KD_OK or KD_ERROR
         */
        ktStatus init(const CkwsSteeringMethodWkPtr& i_smWkPtr);



} ;

/**
   @}
*/

#endif

