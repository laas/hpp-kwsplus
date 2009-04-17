/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
           and Mathieu Poirier (LAAS-CNRS)   
*/


 
#ifndef FLIC_STEERING_METHOD_H
#define FLIC_STEERING_METHOD_H

#warning "deprecated header file. Please include kwsPlus/directPath/flicSteeringMethod.h instead."

/*************************************
INCLUDE
**************************************/

#include <iostream>

#include "KineoWorks2/kwsInterface.h"
#include "KineoUtility/kitInterface.h"
#include "kcd2/kcdInterface.h"
#include "kwsKcd2/kwsKCDBody.h"
#include "KineoWorks2/kwsSteeringMethod.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDefine.h"



/**

\addtogroup flic
@{

*/

/*************************************
STRUCTURES
**************************************/

KIT_PREDEF_CLASS( CflicSteeringMethod );

/*************************************
CLASS
**************************************/
/**
   \brief Steering Method for Flat Interpolation derived from CkwsSteeringMethod.
*/
class CflicSteeringMethod : public CkwsSteeringMethod {
	

 public :
  /**
     \brief   Destructor.
  */
  virtual ~CflicSteeringMethod() ;

  /**
     \brief  create a steering method and return the shared pointer corresponding
     \param  is_oriented : a bool (default = true) ;
     \return a shared pointer on the Steering Method
  */
  static CflicSteeringMethodShPtr create(bool is_oriented = true) ;

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

  /** 
      \brief Constructor.
      \param i_oriented : if true, the steering method will produce oriented paths
  */
  CflicSteeringMethod(bool i_oriented);

  /** 
      \brief Initialization of the CkwsSMLinear object.
      \param         i_smWkPtr : weak pointer to the object itself
      \return        KD_OK or KD_ERROR
  */
  ktStatus init(const CkwsSteeringMethodWkPtr& i_smWkPtr);

 	
 private:

  CkwsSteeringMethodWkPtr   m_weakPtr;	 ///< weak pointer to itself
  bool	 m_oriented;  ///< oriented or not ?
		
} ;


/**
   @}
*/
#endif

