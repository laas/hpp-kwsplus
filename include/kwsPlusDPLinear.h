/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Eiichi Yoshida (AIST/LAAS-CNRS)

*/

#ifndef KWSPLUS_DPLINEAR_H
#define KWSPLUS_DPLINEAR_H

/*************************************
INCLUDE
**************************************/

#include <iostream>
#include "KineoWorks2/kwsDPLinear.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDefine.h"

KIT_PREDEF_CLASS( CkwsConfig );
KIT_PREDEF_CLASS( CkwsSteeringMethod );

KIT_PREDEF_CLASS( CkwsPlusDPLinear );

/**

   \addtogroup kwsplusdplinear
   * @{
   */

// ==============================================================================
//
//  CLASS CkwsPlusDPLinear
//
// ==============================================================================


/**
   \brief Linear direct path with derivative ratio greater than 1.
   \brief Derivative ration will be initialized with a vector of ratios.
*/

class CkwsPlusDPLinear : public CkwsDPLinear {

 public :

  ~CkwsPlusDPLinear();

  /**
     \brief  Create a new instance of a flat Interpolation cart direct path.
     \param  i_start 	: the start configuration
     \param  i_end 	: the end configuration
     \param  i_steeringMethod 	: shared pointer to the instance of the steering method that created the path
     \param inOriented : boolean , direct Path oriented or not
     \return shared pointer to a newly created flicDirectPath
  */
  static CkwsPlusDPLinearShPtr create(const CkwsConfig &i_start,
				  const CkwsConfig &i_end,
				  const CkwsSteeringMethodShPtr &i_steeringMethod);
  
  /**
     \brief  Creates by copy a new instance of a flat Interpolation cart direct path.
     \param i_flatDirectPath : a shared pointer to flicDirectPath that already exist
     \return shared pointer to a newly created flicDirectPath
  */
  static CkwsPlusDPLinearShPtr createCopy (const CkwsPlusDPLinearConstShPtr &i_kwsDPLinear) ; 


  /**
     \brief clone : Returns a shared pointer to a newly allocated copy of the path.
     Note:
     If you want to avoid downcasting the result back to the subclass 
     of the parameter, use the appropriate create() method of the subclass
     so as to create a cloned copy of the object while preserving its type.
     \return shared pointer to a newly allocated copy
  */
  virtual CkwsAbstractPathShPtr clone() const;	

 protected:
	
  /** Constructor.
   *   \param         i_start : the start configuration
   *   \param         i_end : the end configuration
   *   \param         i_steeringMethod : \ref usingSmartPointers "shared pointer" to the instance of the steering method
   *                                     that created the path
   */
  CkwsPlusDPLinear(const CkwsConfig& i_start, const CkwsConfig& i_end, const CkwsSteeringMethodShPtr& i_steeringMethod);
	
  /** Copy constructor.
   *   \param         i_linearDirectPath : the linear direct path to copy
   */
  CkwsPlusDPLinear(const CkwsPlusDPLinear& i_kwsDPLinear);

  /** Initialization.
   *   \param         i_weakPtr : weak pointer to the object itself
   *   \return        #KD_OK | #KD_ERROR
   */
  ktStatus init(const CkwsPlusDPLinearWkPtr& i_weakPtr);

  // inherited -- for doc see parent class
  virtual void interpolate(double i_s, CkwsConfig& o_cfg) const;

  // inherited -- for doc see parent class
  virtual double computePrivateLength() const;

  // inherited -- for doc see parent class
  virtual void										maxAbsoluteDerivative(double i_from, double i_to, std::vector< double >& o_derivative) const;

 private:

  CkwsPlusDPLinearWkPtr	m_weakPtr;		///< weak pointer to itself

};


/**
   @}
*/
#endif

