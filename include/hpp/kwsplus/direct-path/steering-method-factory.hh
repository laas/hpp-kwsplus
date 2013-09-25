/*
  Copyright CNRS-LAAS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUSSTEERINGMETHODFACTORY_H
#define KWSPLUSSTEERINGMETHODFACTORY_H

# include <KineoUtility/kitDefine.h>

KIT_PREDEF_CLASS (CkppSteeringMethodComponent);

/**

\addtogroup smfactory
* @{
*/

/**
   \brief Steering method factory

   This abstract class is intended to be derived to build different types of steering methods.
*/

class CkwsPlusSteeringMethodFactory {
public:
  virtual ~CkwsPlusSteeringMethodFactory() {};
  /**
     \brief Return a steering method
  */
  virtual CkppSteeringMethodComponentShPtr
    makeSteeringMethod(bool inOriented) = 0;
};


/**
   @}
*/

#endif
