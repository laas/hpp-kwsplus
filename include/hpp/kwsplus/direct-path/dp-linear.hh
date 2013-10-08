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
#include <hpp/kwsplus/direct-path/direct-path.hh>

HPP_KIT_PREDEF_CLASS( CkwsConfig );
HPP_KIT_PREDEF_CLASS( CkwsSteeringMethod );

HPP_KIT_PREDEF_CLASS( CkwsPlusDPLinear );

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
   \brief Extension of KineoWorks linear direct path.

*/

class CkwsPlusDPLinear : public CkwsPlusDirectPath {

 public :

  ~CkwsPlusDPLinear();

  /**
     \brief  Create a new instance of a flat Interpolation cart direct path.
     \param  inStart 	: the start configuration
     \param  inEnd 	: the end configuration
      \param inRatioVector The vector of ratios used to overestimate 
      variations of degree-of freedom over intervals 
      (see maxAbsoluteDerivative()).
     \param  inSteeringMethod 	: shared pointer to the instance of the steering method that created the path

     \return shared pointer to a newly created flicDirectPath
  */
  static CkwsPlusDPLinearShPtr create(const CkwsConfig &inStart,
				      const CkwsConfig &inEnd,
				      const std::vector<double> &inRatioVector,
				      const CkwsSteeringMethodShPtr &inSteeringMethod);
  
  /**
     \brief  Copy a direct path of type CkwsPlusDPLinear.
     \param inKwsDPLinear input direct path
     \return shared pointer to a newly created direct path.
  */
  static CkwsPlusDPLinearShPtr createCopy (const CkwsPlusDPLinearConstShPtr &inKwsDPLinear) ; 


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
	
  /** 
      \brief Constructor.
      \param         inStart  The start configuration
      \param         inEnd  The end configuration
      \param inRatioVector The vector of ratios used to overestimate 
      variations of degree-of freedom over intervals 
      (see maxAbsoluteDerivative()).
      \param         inSteeringMethod Shared pointer to the instance 
      of the steering method that created the path.
   */
  CkwsPlusDPLinear(const CkwsConfig& inStart, const CkwsConfig& inEnd, 
		   const std::vector<double> &inRatioVector, 
		   const CkwsSteeringMethodShPtr& inSteeringMethod);
	
  /** Copy constructor.
      \param         inKwsDPLinear the linear direct path to copy
   */
  CkwsPlusDPLinear(const CkwsPlusDPLinear& inKwsDPLinear);

  /** 
      \brief Initialization
      \param         inWeakPtr : weak pointer to the object itself
      \return        KD_OK | KD_ERROR
   */
  ktStatus init(const CkwsPlusDPLinearWkPtr& inWeakPtr);


  /**
     \brief Interpolate between initial and end config.

     Interpolation does not take into account extraction and reversion.
  */
  virtual void interpolate (double inParam, CkwsConfig &outConfig) const;

  /**
     \brief Returns the parameter range of the direct path at the time it was built.
  */
  virtual double computePrivateLength() const;

  /**
     \brief Return an overestimate of the absolute value of the derivative 
     vector of the direct path between two positions on the path.

     CkwsDPLinear parent implementation is called. Each value of outDerivative is
     then multiplied by value of same rank in ratio vector passed at construction.

     \note if size of ratio vector is smaller than size of device, missing values
     are replaced by 1.
  */
  virtual void	maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double>& outDerivative) const;

  /**
     \brief Compute the velocity before extraction and reversion
     
     \param inDistance Distance along the path
     \retval outVelocity Derivative of the path with respect to distance parameter.
     \return KD_OK | KD_ERROR
     
  */
  ktStatus getVelocityAtDistanceAtConstruction(double inDistance, 
					       std::vector<double>& outVelocity) const;

 private:

  CkwsPlusDPLinearWkPtr	attWeakPtr;		///< weak pointer to itself

  std::vector<double> attRatioVector; /// <ratio of vector with respect to linear max absolute derivative

  /**
     \brief Store linear direct path
  */
  CkwsDPLinearShPtr attLinearDP;
};


/**
   @}
*/
#endif

