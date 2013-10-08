/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
           and Mathieu Poirier (LAAS-CNRS)   
*/


 
#ifndef FLIC_STEERING_METHOD_H
#define FLIC_STEERING_METHOD_H

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

#include <hpp/util/kitelab.hh>

/**

\addtogroup flic
@{

*/

/*************************************
STRUCTURES
**************************************/

HPP_KIT_PREDEF_CLASS( CflicSteeringMethod );

/*************************************
CLASS
**************************************/
/**
   \brief Steering Method for Flat Interpolation derived from CkwsSteeringMethod.
*/
class CflicSteeringMethod : public CkwsSteeringMethod {
	

 public :

	/** The following macro declares:
     static const CkitClassShPtr CLASS;
     virtual CkitClassShPtr classObject() const;
	*/
	HPP_KIT_DECLARE_CLASS();

  /**
     \brief   Destructor.
  */
  virtual ~CflicSteeringMethod() ;

  // Interfaces for object coding/decoding:
  virtual void encodeWithCoder(const CkitCoderShPtr& i_coder) const;
  virtual ktStatus initWithCoder(const CkitCoderShPtr& i_coder, const CkitCodableShPtr& i_self);

  /**
     \brief  create a steering method and return the shared pointer corresponding
     \param  is_oriented : a bool (default = true) ;
     \return a shared pointer on the Steering Method
  */
  static CflicSteeringMethodShPtr create(bool is_oriented = true) ;

  /**
     \brief  create a steering method and return the shared pointer corresponding
     \param  is_oriented : a bool (default = true) ;
     \return a shared pointer on the Steering Method
  */
  static CflicSteeringMethodShPtr createCopy
    (const CflicSteeringMethodConstShPtr& steeringMethod) ;

  /**
     Factory method that creates a new direct path between two configurations.
     \param i_startCfg : the start config
     \param i_endCfg : the end config
     \return a shared pointer on the direct path just create
  */
  virtual CkwsDirectPathShPtr makeDirectPath (const CkwsConfig& i_startCfg,
					      const CkwsConfig& i_endCfg) const;
 	
  /**
     Returns whether the steering method produces oriented or non-oriented paths for a given space.
     \return true if the steering method is oriented or false
  */
  virtual bool isOriented(const CkwsConfigSpaceConstShPtr &i_space) const ;

  /**
     Returns whether the steering method produces oriented or non-oriented paths
     \return true if the steering method is oriented or false
  */
  virtual bool isOriented() const ;

  virtual CkwsSteeringMethodShPtr clone() const;

 protected:
	/** 
		\brief constructor
		\note All codable objects must have a default constructor
	*/
  CflicSteeringMethod();	

  /** 
      \brief Constructor.
      \param i_oriented : if true, the steering method will produce oriented paths
  */
  CflicSteeringMethod(bool i_oriented);
  CflicSteeringMethod(const CflicSteeringMethod& steeringMethod);

  /** 
      \brief Initialization of the CkwsSMLinear object.
      \param         i_smWkPtr : weak pointer to the object itself
      \return        KD_OK or KD_ERROR
  */
  ktStatus init(const CflicSteeringMethodWkPtr& i_smWkPtr);

 	
 private:

  CflicSteeringMethodWkPtr   m_weakPtr;	 ///< weak pointer to itself
  bool	 m_oriented;  ///< oriented or not ?
		
} ;


/**
   @}
*/
#endif

