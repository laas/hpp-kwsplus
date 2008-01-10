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

using namespace std ;

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

class CkwsPlusSMLinear : public CkwsSteeringMethod {

 public :
  /**
     \brief   Destructor.
  */
  virtual ~CkwsPlusSMLinear() ;

  /** Static function that creates an instance of the CkwsSMLinear class.
   *   \param         i_oriented : if true, the steering method will produce oriented paths
   *   \param         i_ratio_vector : the vector of derivative ratio with respect to \\
   *                  the linear SM for each DoF to calculate maxAbsoluteDerivative
   *   \return        \ref usingSmartPointers "shared pointer" to the newly created steering method
   */
  static CkwsPlusSMLinearShPtr create(const std::vector<double> &i_ratio_vector,
				      bool i_oriented = false);

  // inherited -- for doc see parent class
  virtual CkwsDirectPathShPtr makeDirectPath(const CkwsConfig& i_startCfg, const CkwsConfig& i_endCfg);

  // inherited -- for doc see parent class
  virtual bool isOriented() const;

 protected:

  /** Constructor.
   *   \param         i_oriented : if true, the steering method will produce oriented paths
   *   \param         i_ratio_vector : the vector of derivative ratio with respect to \\
   *                  the linear SM for each DoF to calculate maxAbsoluteDerivative
   */
  CkwsPlusSMLinear(const std::vector<double> &i_ratio_vector, bool i_oriented);

  /** Initialization of the CkwsPlusSMLinear object.
   *   \param         i_smWkPtr : weak pointer to the object itself
   *   \return        #KD_OK | #KD_ERROR
   */
  ktStatus init(const CkwsSteeringMethodWkPtr& i_smWkPtr);

 private:

  CkwsSteeringMethodWkPtr m_weakPtr;	 ///< weak pointer to the object itself
  bool m_oriented;		///< does the steering method produce oriented paths?
  std::vector<double> m_ratio_vector; ///<ratio of vector with respect to linear max absolute derivative

};

/**
   @}
*/

#endif
