#include <iostream>
#include "kwsIPPRdmBuilder2.h"

#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoWorks2/kwsPath.h"
#include "KineoWorks2/kwsRoadmap.h"


CkwsIPPRdmBuilder2ShPtr CkwsIPPRdmBuilder2::create(const CkwsRoadmapShPtr &inRoadmap, 
						   double inPenetration, 
						   const CkwsDiffusionNodePickerShPtr &inPicker, 
						   const CkwsDiffusionShooterShPtr &inShooter)
{
  CkwsIPPRdmBuilder2* rdmBuilderPtr = new CkwsIPPRdmBuilder2(inRoadmap);
  CkwsIPPRdmBuilder2ShPtr rdmBuilderShPtr(rdmBuilderPtr);
  CkwsIPPRdmBuilder2WkPtr rdmBuilderWkPtr(rdmBuilderShPtr);

  if (rdmBuilderPtr->init(rdmBuilderWkPtr, inPicker, inShooter) != KD_OK) {
    rdmBuilderShPtr.reset();
  }
  rdmBuilderPtr->penetration(inPenetration);
  std::cout << "CkwsIPPRdmBuilder2::create" << std::endl;
  return rdmBuilderShPtr;
}

ktStatus CkwsIPPRdmBuilder2::init(const CkwsIPPRdmBuilder2WkPtr &inWeakPtr, 
				  CkwsDiffusionNodePickerShPtr inPicker, 
				  CkwsDiffusionShooterShPtr inShooter)
{
  if (CkwsIPPRdmBuilder::init(inWeakPtr, inPicker, inShooter) != KD_OK) {
    return KD_ERROR;
  }
  attWeakPtr = inWeakPtr;
  return KD_OK;
}


CkwsIPPRdmBuilder2::CkwsIPPRdmBuilder2(const CkwsRoadmapShPtr& inRoadmap) :
  CkwsIPPRdmBuilder(inRoadmap)
{
}

ktStatus CkwsIPPRdmBuilder2::buildOneStep()
{
  ktStatus success = CkwsIPPRdmBuilder::buildOneStep();
  bool pathFound;
  CkwsNodeConstShPtr connectedStart;
  CkwsNodeConstShPtr connectedGoal;
  CkwsPathShPtr	foundPath;
  std::list< CkwsEdgeShPtr >edges;

  if(isProblemSolved(connectedStart, connectedGoal))
    {
      std::cout << "isProblemSolved()" << std::endl;
      
      pathFound = roadmap()->findPath(connectedStart, connectedGoal,
				      builderDistance(), foundPath, edges);
      
      if(!pathFound)
	{
	  std::cout << "ERROR: !pathFound" << std::endl;
	}
      if(!foundPath)
	{
	  std::cout << "ERROR: !foundPath" << std::endl;
	}
    }
  
  return success;
}

