/*
   Copyright (c) JRL CNRS-AIST,
   @author : Florent Lamiraux

*/

#ifndef KWSPLUSSTOPRDMBUILDERDELEGATE_H
#define KWSPLUSSTOPRDMBUILDERDELEGATE_H

#include <iostream>
#include "KineoWorks2/kwsRdmBuilderDelegate.h"

/**
   \addtogroup kwsPlusEnhancedRoadmapManagement
   @{
*/

/**
   \brief Delegate that interrupts a roadmap builder

   To use this delegate, pass it to a roadmap builder using
   CkwsPathPlanner::addDelegate().

   To interrupt the roadmap builder during planning, call 
   \code shouldStop(true);\endcode.
*/

class CkwsPlusStopRdmBuilderDelegate : public CkwsRdmBuilderDelegate 
{
public:
  /**
     \brief Contructor
  */
  CkwsPlusStopRdmBuilderDelegate() : attShouldStop(false) 
  {
#if DEBUG==2   
    attNbInstances++;
    std::cout << "CkwsPlusStopRdmBuilderDelegate: nb instances = " << attNbInstances << std::endl;
#endif
  };
  
  virtual ~CkwsPlusStopRdmBuilderDelegate()
  {
#if DEBUG==2
    attNbInstances--;
    std::cout << "CkwsPlusStopRdmBuilderDelegate: nb instances = " << attNbInstances << std::endl;
#endif
  }

  /**
     \brief Set stop condition.
  */
  void shouldStop(bool inShouldStop) 
  { 
    attShouldStop = inShouldStop;
  };
  
  /**
     \brief Called repeatedly during the path planning process to allow interruption.
     
  */
  virtual bool plannerShouldStopPlanning(const CkwsPathPlannerConstShPtr &inPlanner) 
  {
    if (CkwsRdmBuilderDelegate::plannerShouldStopPlanning(inPlanner)) {
      return true;
    }
    if (attShouldStop) {
      attShouldStop = false;
      return true;
    }
    return false;
  }
  
  
private:
  
  /**
     \brief Flag storing whether the planner should stop.
  */
  bool attShouldStop;
  
#if DEBUG==2
  static unsigned int attNbInstances;
#endif
};

/**
   @}
*/

#endif
