#ifndef KWSPLUS_DIRECTPATH_H
#define KWSPLUS_DIRECTPATH_H

#include "KineoWorks2/kwsDirectPath.h"
#include <iostream>

KIT_PREDEF_CLASS(CkwsPlusDirectPath);

/**
   \addtogroup kwsPlus_DP

   @{
*/

/**
   \brief Specialization of CkwsDirectPath that manages extraction and reversion.
*/

class CkwsPlusDirectPath : public CkwsDirectPath {
public:
  virtual ~CkwsPlusDirectPath();

  /**
     \brief Returns a shared pointer to a newly allocated copy of the path.
  */
  virtual CkwsAbstractPathShPtr clone () const = 0;

  /**
     \name Reversion and extraction management.
     @{
  */
  /**
     \brief Returns whether direct path has been reversed.
  */
  bool isReversed() const {return attReverse;};

  /**
     \brief Return beginning of extracted interval.

     A new path can be created by restriction to an interval. If this path comes from such a restriction,
     this attribute is the lower bound of the interval.
  */
  const double& uStart() const {return attUstart;};

  /**
     \brief Return end of extracted interval.

     A new path can be created by restriction to an interval. If this path comes from such a restriction,
     this attribute is the upper bound of the interval.
  */
  const double& uEnd() const {return attUend;};

  /**
     \brief Reimplementation from CkwsDirectPath in order to keep track of the extracted interval.
  */
  virtual ktStatus extractFrom(double inParam);

  /**
     \brief Reimplementation from CkwsDirectPath in order to keep track of the extracted interval.
  */
  virtual ktStatus extractTo(double inParam);

 
  /**
     \brief Reimplementation from CkwsDirectPath in order to keep track of the extracted interval.
  */
  virtual ktStatus reverse();

  /**
     \brief Get the private start configuration for directPath reproduction.
  */
  const CkwsConfig& getPrivateStart () const; 

  /**
     \brief Get the private start configuration for directPath reproduction.
  */
  const CkwsConfig& getPrivateEnd () const; 

  /**
     \brief Get the private length for directPath reproduction.
  */
  double getPrivateLength () const; 

  /**
     @}
  */

  /**
     \name Velocity 
     @{
  */
  /**
     \brief Compute derivative of configuration along direct path

     \param inParameter parameter along direct path
     \retval outVelocity derivative of each degree of freedom along direct path.

     Derivative is computed with respect to length parameterization.
  */
  ktStatus getVelocityAtDistance(double inParameter, std::vector<double>& outVelocity) const;

  /**
     @}
  */
protected:
  /**
     \brief Constructor by end configuration and steering method.
  */
  CkwsPlusDirectPath(const CkwsConfig& inStartConfig, 
		     const CkwsConfig& inEndConfig,
		     const CkwsSteeringMethodShPtr& inSteeringMethod);

  /**
     \brief Copy constructor
  */
  CkwsPlusDirectPath(const CkwsPlusDirectPath& inDirecPath);

  /**
     Initialization of the object
     Calls CkwsDirectPath::init and updates attUend.
  */
  ktStatus init(const CkwsPlusDirectPathWkPtr& inWeakPtr);

  /**
     \brief Return the length of this direct path.
     
     Length is the sum of the lengths of each direct path in vector.
  */
  virtual double computePrivateLength() const = 0;

  /**
     \brief Compute configuration at given length on direct path.
  */
  virtual void interpolate(double s, CkwsConfig &outCfg) const = 0;

  /**
     \brief Compute the maximal absolute values of derivative of each degree of freedom on given interval.
  */
  virtual void maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double> &outVectorDeriv) const = 0;

  /**
     \brief Compute the velocity before extraction and reversion

     \param inDistance Distance along the path
     \retval outVelocity Derivative of the path with respect to distance parameter.
     \return KD_OK | KD_ERROR

  */
  virtual ktStatus getVelocityAtDistanceAtConstruction(double inDistance, std::vector<double>& outVelocity) const;

private:
  /**
     \brief Beginning of extracted interval.
     A new path can be created by restriction to an interval. If this path comes from such a restriction,
     this attribute is the lower bound of the interval.
  */
  double attUstart;

  /**
     \brief End of extracted interval.
     A new path can be created by restriction to an interval. If this path comes from such a restriction,
     this attribute is the upper bound of the interval.
  */
  double attUend;

  /**
     \brief Whether path is reversed.
     Store information about the fact that CkwsDirectPath has been reversed.
     This information is used when writing the direct path in a file.
  */
  bool attReverse;
};

/**
   @}
*/

#endif
