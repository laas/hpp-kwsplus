/*
  Copyright CNRS 2008

  Author: Florent Lamiraux
*/

#ifndef KWSPLUS_DIFFUSIONNODEPICKER_FACTORY_H
#define KWSPLUS_DIFFUSIONNODEPICKER_FACTORY_H

#include "KineoWorks2/kwsPickerBasic.h"
#include "KineoWorks2/kwsPickerSmallestTree.h"

KIT_PREDEF_CLASS(CkwsDiffusionNodePicker);

/**

\addtogroup diffusionNodePickerFactory
* @{
*/

/**
   \brief Diffusion node picker factory

   This abstract class is intended to be derived to build different 
   types of diffusion node pickers (see KineoWorks documentation).
*/

class CkwsPlusDiffusionNodePickerFactory {
public:
  virtual ~CkwsPlusDiffusionNodePickerFactory(){};

  /**
     \brief Build a diffusion node picker
  */
  virtual CkwsDiffusionNodePickerShPtr makeDiffusionNodePicker() = 0;
};

/**
   \brief Basic diffusion node picker factory
*/

class CkwsPlusBasicDiffusionNodePickerFactory :
  public CkwsPlusDiffusionNodePickerFactory
{
  CkwsDiffusionNodePickerShPtr makeDiffusionNodePicker()
  {
    return CkwsPickerBasic::create();
  };
};

/**
   \brief Smallest Tree diffusion node picker factory
*/

class CkwsPlusSmallestTreeDiffusionNodePickerFactory :
  public CkwsPlusDiffusionNodePickerFactory
{
  CkwsDiffusionNodePickerShPtr makeDiffusionNodePicker()
  {
    return CkwsPickerSmallestTree::create();
  };
};



/**
   @}
*/


#endif
