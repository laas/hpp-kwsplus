/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)

*/

#include <iostream>

#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "kwsPlusDrawRdmBuilderDelegate.h"
#include "kwsPlusRoadmap.h"

void CkwsPlusDrawRdmBuilderDelegate::willStartBuilding(const CkwsRoadmapBuilderShPtr& inRdmBuilder, 
						       CkwsPathShPtr& inOutPath)
{
  CkwsPlusRoadmapShPtr roadmap = KIT_DYNAMIC_PTR_CAST(CkwsPlusRoadmap, inRdmBuilder->roadmap());

  if (roadmap) {
    roadmap->display();
  }
}

void CkwsPlusDrawRdmBuilderDelegate::didAddEdge(const CkwsRoadmapBuilderConstShPtr &inRdmBuilder,
						const CkwsEdgeConstShPtr & inEdge)
{
  CkwsPlusRoadmapShPtr roadmap = KIT_DYNAMIC_PTR_CAST(CkwsPlusRoadmap, inRdmBuilder->roadmap());

  if (roadmap) {
    roadmap->lastEdge();
  }
}

void CkwsPlusDrawRdmBuilderDelegate::didAddNode(const CkwsRoadmapBuilderConstShPtr& inRdmBuilder,
						const CkwsNodeConstShPtr& inNode)
{
  CkwsRoadmapShPtr roadmap = inRdmBuilder->roadmap();
  if (roadmap) {
    std::cout << "Number of nodes: " << roadmap->countNodes() << std::endl; 
  }
}

bool CkwsPlusDrawRdmBuilderDelegate::shouldStopBuilding(const CkwsRoadmapBuilderConstShPtr& inRdmBuilder)
{
  CkwsPlusRoadmapShPtr roadmap = KIT_DYNAMIC_PTR_CAST(CkwsPlusRoadmap, inRdmBuilder->roadmap());

  if (roadmap) {
    if (roadmap->countNodes() > 1000) {
      roadmap->compute();
      return true;
    }
  }
  return false;
}

CkitProgressDelegateShPtr CkwsPlusDrawRdmBuilderDelegate::clone() const
{
  CkwsPlusDrawRdmBuilderDelegate* copy = new CkwsPlusDrawRdmBuilderDelegate(*this);
  return CkitProgressDelegateShPtr(copy);
}
