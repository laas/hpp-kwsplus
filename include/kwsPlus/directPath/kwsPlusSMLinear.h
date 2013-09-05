/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Eiichi Yoshida (AIST/LAAS-CNRS)

*/

#ifndef KWSPLUS_SMLINEAR_H
#define KWSPLUS_SMLINEAR_H

/*************************************
INCLUDE
**************************************/

#include <iostream>

#include "KineoWorks2/kwsSteeringMethod.h"

#include "KineoUtility/kitInterface.h"
#include "KineoWorks2/kwsDefine.h"


/**

   \addtogroup kwsplusdplinear
   * @{
   */

/*************************************
STRUCTURES
**************************************/

KIT_PREDEF_CLASS( CkwsConfig );
KIT_PREDEF_CLASS( CkwsPlusSMLinear );

/*************************************
CLASS
**************************************/

/**
   \brief Steering method producing CkwsPlusDPLinear direct paths

*/

class CkwsPlusSMLinear : public CkwsSteeringMethod {

 public :
	 
	// The following macro declares:
  //   static const CkitClassShPtr CLASS;
  //   virtual CkitClassShPtr classObject() const;
  KIT_DECLARE_CLASS();
	
  /**
     \brief   Destructor.
  */
  virtual ~CkwsPlusSMLinear() ;

  /** Create an instance of the CkwsSMLinear class.
      \param         inOriented : if true, the steering method will produce oriented paths
      \param         inRatioVector The vector of ratios used to overestimate 
      variations of degree-of freedom over intervals 
      (see CkwsPlusDPLinear::maxAbsoluteDerivative()).
      \return        Shared pointer to the newly created steering method
   */
  static CkwsPlusSMLinearShPtr create
    (const std::vector<double> &inRatioVector=std::vector<double>(0),
     bool inOriented = false);
  
  /// Clone the steering method
  virtual CkwsSteeringMethodShPtr clone() const;

  /// Create an new instance copy of the CkwsSMLinear class.
  static CkwsPlusSMLinearShPtr createCopy (const CkwsPlusSMLinearConstShPtr&
					   sm);
	
  // inherited -- for doc see parent class
  // Interfaces for object coding/decoding:
  virtual void encodeWithCoder(const CkitCoderShPtr& i_coder) const;
  virtual ktStatus initWithCoder(const CkitCoderShPtr& i_coder, const CkitCodableShPtr& i_self);
  virtual CkwsDirectPathShPtr makeDirectPath
    (const CkwsConfig& inStartConfig, const CkwsConfig& inEndConfig) const;

  // inherited -- for doc see parent class
  virtual bool isOriented(const CkwsConfigSpaceConstShPtr& configSpace) const;

  void getRatioVector (std::vector<double>& o_ratioVect) const;
 protected:

  /** Constructor.
      \param         inOriented : if true, the steering method will produce oriented paths
      \param         inRatioVector The vector of ratios used to overestimate 
      variations of degree-of freedom over intervals 
      (see CkwsPlusDPLinear::maxAbsoluteDerivative()).
   */
  CkwsPlusSMLinear(const std::vector<double> &inRatioVector, bool inOriented);

  /// Copy constructor
  CkwsPlusSMLinear(const CkwsPlusSMLinear& sm);

  /** Initialization of the CkwsPlusSMLinear object.
   *   \param         inSmWkPtr : weak pointer to the object itself
   *   \return        KD_OK | KD_ERROR
   */
  ktStatus init(const CkwsPlusSMLinearWkPtr& inSmWkPtr);

 private:

  /**
     \brief Weak pointer to itself
  */
  CkwsPlusSMLinearWkPtr attWeakPtr;	 

  /**
     \brief whether steering method produces oriented paths
  */
  bool attOriented;		

  /**
     \brief Vector of security ratios that are multiplied to result of 
     CkwsLinear::maxAbsoluteDerivative.
  */
  std::vector<double> attRatioVector; 

};

/**
   @}
*/

#endif
