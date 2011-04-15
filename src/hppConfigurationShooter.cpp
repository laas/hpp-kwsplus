/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Luis Delgado (LAAS-CNRS)

*/


#include "kwsPlus/roadmap/hppConfigurationShooter.h"
#include "KineoWorks2/kwsNode.h"
#include <iostream>

// ==========================================================================

  ChppConfigurationShooterShPtr ChppConfigurationShooter::create()
  {
	ChppConfigurationShooter* ptr= new ChppConfigurationShooter();
	ChppConfigurationShooterShPtr shPtr(ptr);
	if(shPtr->init()!=KD_OK)
	{
	  shPtr.reset();
	}
	return shPtr;
  }



// ==========================================================================

  bool ChppConfigurationShooter::shoot(const CkwsNodeShPtr& i_node,
                                                 CkwsConfig& o_cfg)
  {
    o_cfg=CkwsConfig(i_node->config());
    o_cfg.randomize();
    return true;
  }


// ==========================================================================

  ktStatus ChppConfigurationShooter::init()
  {
    return KD_OK;
  }


