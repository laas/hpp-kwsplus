/*
  Copyright CNRS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUS_DIFFUSIONSHOOTER_FACTORY_H
#define KWSPLUS_DIFFUSIONSHOOTER_FACTORY_H

#include "KineoWorks2/kwsShooterConfigSpace.h"
#include "KineoWorks2/kwsShooterRoadmapBox.h"
#include "KineoWorks2/kwsShooterRoadmapNodes.h"

KIT_PREDEF_CLASS(CkwsDiffusionShooter);

/**

\addtogroup diffusionShooterFactory
* @{
*/

/**
   \brief Diffusion shooter factory

   This abstract class is intended to be derived to build different 
   types of diffusion shooters (see KineoWorks documentation).
*/

class CkwsPlusDiffusionShooterFactory {
public:
  virtual ~CkwsPlusDiffusionShooterFactory(){};

  /**
     \brief Build a diffusion shooter
  */
  virtual CkwsDiffusionShooterShPtr makeDiffusionShooter() = 0;
};


/**
   \brief  config space diffusion shooter factory
*/

class CkwsPlusShooterConfigSpaceFactory :
  public CkwsPlusDiffusionShooterFactory
{
  CkwsDiffusionShooterShPtr makeDiffusionShooter()
  {
    CkwsDiffusionShooterShPtr shooter = CkwsShooterConfigSpace::create();
    return shooter;
  };
};

/**
   \brief  Roadmap box diffusion shooter factory
*/

class CkwsPlusShooterRoadmapBoxFactory :
  public CkwsPlusDiffusionShooterFactory
{
  CkwsDiffusionShooterShPtr makeDiffusionShooter()
  {
    CkwsDiffusionShooterShPtr shooter = CkwsShooterRoadmapBox::create();
    return shooter;
  };
};

/**
   \brief  Roadmap nodes diffusion shooter factory
*/

class CkwsPlusShooterRoadmapNodesFactory :
  public CkwsPlusDiffusionShooterFactory
{
  CkwsDiffusionShooterShPtr makeDiffusionShooter()
  {
    CkwsDiffusionShooterShPtr shooter = CkwsShooterRoadmapNodes::create();
    return shooter;
  };
};



/**
   @}
*/


#endif
