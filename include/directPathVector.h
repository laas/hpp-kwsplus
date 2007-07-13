/*
  Copyright 2007 Florent Lamiraux
*/

#ifndef DIRECT_PATH_VECTOR_H
#define DIRECT_PATH_VECTOR_H

#include <iostream>
#include <fstream>
#include <vector>
#include "KineoWorks2/kwsDirectPath.h"

KIT_PREDEF_CLASS(CdirectPathVector);

/**
   \brief Implement a vector of direct paths as a direct path.

   Some steering methods build direct paths as a vector of elementary
   direct paths.  For instance, a steering method for a car like
   vehicle going forward and backward can return one or two direct
   paths if a cusp configuration is inserted. 

*/

class CdirectPathVector : public CkwsDirectPath {
public:
  /*

    P U B L I C   M E T H O D S 

  */

  /**
     \name Creation and copy.
     @{
  */
  /**
     \brief Creates a vector of direct path and returns a shared pointer to it.
  */
  static CdirectPathVectorShPtr create(const CkwsConfig& inStartCfg, const CkwsConfig& inEndCfg,
				       const CkwsSteeringMethodShPtr& inSteeringMethod,
				       const std::vector<CkwsDirectPathConstShPtr>& inDirectPathVector);

  /**
     \brief Create a copy of this direct path.
  */
  CkwsAbstractPathShPtr clone() const; 

  /**
     @}
  */

  /**
     \name Access to direct paths.
     @{
  */
  /**
     \brief Return number of direct paths in vector
  */
  unsigned int countDirectPaths() const {return attDirectPathVector.size();};

  /**
     \brief Access to direct paths of vector
  */
  CkwsDirectPathConstShPtr directPathAtRank(unsigned int inRank) const {return attDirectPathVector[inRank];};

  /**
     @}
  */

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
     @}
  */

protected:

  /*

    P R O T E C T E D    M E T H O D S 

  */


 /**
     \brief  Create by copy a new instance of a direct path vector
  */
  static CdirectPathVectorShPtr createCopy (const CdirectPathVectorConstShPtr &inDirectPathVector); 

  /**
     \brief Constructor with vector of direct paths.
  */
  CdirectPathVector(const CkwsConfig& inStartConfig, const CkwsConfig& inGoalConfig, 
		    const CkwsSteeringMethodShPtr& inSteeringMethod,
		    std::vector<CkwsDirectPathConstShPtr> inDirectPathVector);

  /**
     \brief Copy Constructor.
  */
  CdirectPathVector(const CdirectPathVector& inDirectPathVector);

  /**
     Initialization of the object
     Mainly calls CkwsDirectPath::init and stores weak pointer to object.
  */
  ktStatus init(const CdirectPathVectorWkPtr& inWeakPtr);

  /**
     \brief Return the length of this direct path.
     
     Length is the sum of the lengths of each direct path in vector.
  */
  double computePrivateLength() const {return attLength;};

  /**
     \brief Compute configuration at given length on direct path.
  */
  void interpolate(double inLength, CkwsConfig& outConfig) const;

  /**
     \brief Compute the maximal absolute values of derivative of each degree of freedom on given interval.
  */
  void maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double>& outDerivatives) const;

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

private:
  /*

    P R I V A T E    M E T H O D S 

  */

  /**
     \brief Insert a direct path at the end of this one.
  */
  ktStatus addDirectPath(CkwsDirectPathShPtr inDirectPath);

  /**
     \brief Compute the max between two vector, coordinate by coordinate
     Result is put in first vector.
  */
  void computeMaxOfTwoVectors(std::vector<double>& inOutVector1, const std::vector<double>& inVector2) const;

  /*

    P R I V A T E   A T T R I B U T T E S

  */
  /**
     \brief Weak pointer to itself.

     Used to create a shared pointer to itself in clone().
  */
  CdirectPathVectorWkPtr attWeakPtr;

  /**
     \brief Vector of direct paths constituting this one.
  */
  std::vector<CkwsDirectPathConstShPtr> attDirectPathVector;

  /**
     \brief Length of local path
     Length is the sum of the lengths of each direct path in vector.
  */
  double attLength;

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
#endif
