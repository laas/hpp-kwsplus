
#include <vector>
#include <iostream>

#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsSMLinear.h"
#include "KineoWorks2/kwsDirectPath.h"
#include "KineoWorks2/kwsDPLinear.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoWorks2/kwsDof.h"
#include "KineoWorks2/kwsConnectedComponent.h"
#include "KineoWorks2/kwsSMLinear.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsValidatorCfgCollision.h"
#include "KineoWorks2/kwsValidatorDPCollision.h"
#include "KineoWorks2/kwsValidatorPathCollision.h"
#include "KineoWorks2/kwsNodeFactory.h"

#include <hpp/kwsplus/roadmap/vis-rdm-builder.hh>
#include <hpp/kwsplus/roadmap/shooter-active-dof.hh>

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cout << "ChppvisRdmBuilder:" << x << std::endl
#define ODEBUG1(x) std::cerr << "ChppvisRdmBuilder:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "ChppvisRdmBuilder:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif


// ==========================================================================

ChppVisRdmBuilderShPtr ChppVisRdmBuilder::create
(const CkwsRoadmapShPtr &inRoadmap,
 double inPenetration,
 const ChppShooterActiveDofShPtr &inShooter)
{

  ChppVisRdmBuilder* rdmBuilderPtr = new ChppVisRdmBuilder(inRoadmap);
  ChppVisRdmBuilderShPtr rdmBuilderShPtr(rdmBuilderPtr);
  ChppVisRdmBuilderWkPtr rdmBuilderWkPtr(rdmBuilderShPtr);

  if (rdmBuilderPtr->init(rdmBuilderWkPtr,inShooter) != KD_OK){
    rdmBuilderShPtr.reset();
  }
  else {
    if(KD_ERROR == CkwsValidatorDPCollision::setPenetration
       (rdmBuilderShPtr->configSpace ()->directPathValidators(), 
	inPenetration)) {
      ODEBUG1(" Unvalid penetration: " << inPenetration);
      rdmBuilderShPtr.reset();
    }
  }

  return rdmBuilderShPtr;

}

// ==========================================================================

ChppVisRdmBuilder::ChppVisRdmBuilder(const CkwsRoadmapShPtr& inRoadmap)
  : CkwsRoadmapBuilder(inRoadmap), att_max_iterations(2000)
{}

// ==========================================================================

ktStatus ChppVisRdmBuilder::init(const ChppVisRdmBuilderWkPtr& inWeakPtr,
                                 const ChppShooterActiveDofShPtr &inShooter)
{
  ktStatus success = CkwsRoadmapBuilder::init(inWeakPtr);
  att_n_iterations = 0;
  att_shooter=inShooter;

#if 0 // Florent: I do not understand the following lines
  this->roadmap()->device()->userDirectPathValidators()->add(CkwsValidatorDPCollision::create(this->roadmap()->device(),0.0001));
  this->roadmap()->device()->userPathValidators()->add(CkwsValidatorPathCollision::create(this->roadmap()->device(),0.0001));
  this->roadmap()->device()->userConfigValidators()->add(CkwsValidatorCfgCollision::create (this->roadmap()->device()));
#endif

  return success ;
}


// ==========================================================================

ktStatus ChppVisRdmBuilder::buildOneStep()
{
  CkwsConfig currentConf(this->roadmap()->device());
  CkwsConfig config(this->roadmap()->device());
  this->roadmap()->device()->getCurrentConfig(currentConf);
  CkwsNodeShPtr currentNode = nodeFactory()->makeNode(currentConf);


  std::vector<double> dofs;
  currentConf.getDofValues(dofs);

  CkwsValidatorSetShPtr constraints =
    this->roadmap()->device()->userConstraints();
  constraints->isActivated(false);

  if(att_shooter->shoot(currentNode,config))
  {
    constraints->isActivated(true);
    constraints->apply(config);
    bool val=true;
    CkwsValidatorCfgCollisionShPtr cfv =
      CkwsValidatorCfgCollision::create
      (this->roadmap()->device()->configSpace ());
    CkwsNodeShPtr node = nodeFactory()->makeNode(config);
    unsigned int i=0;
    while( i<constraints->count() && val)
    {
      val=val && constraints->at(i)->validate(config);
      i++;
    }
    if(val && cfv->validate(config))
    {
      att_n_iterations++;
      addVisibilityNode(node);
    }
  }
  else {
    ODEBUG1("::buildOneStep: failed to shoot a configuration.");
    return KD_ERROR;
  }
  return KD_OK;
}


// ==========================================================================

void ChppVisRdmBuilder::addVisibilityNode(CkwsNodeShPtr node)
{
  nbNearestNodes(10);
  if(roadmap()->countNodes()<2)
  {
    if(addNode(node)!=KD_ERROR)
    {
      link(CkwsRoadmapBuilder::BIDIRECTIONAL,node,
	   CkitParameterMap::create ());
    }
  }
  else
  {
    int ncc=0;

    CkwsNodeShPtr o_node = nodeFactory()->makeNode(node->config());
    CkwsDirectPathShPtr o_dp;
    ODEBUG2(" nbr cc " << roadmap()->countConnectedComponents());

    for(unsigned int i=0; i<roadmap()->countConnectedComponents(); i++)
    {
      CkwsConnectedComponentShPtr connectedComponent;
      roadmap()->getConnectedComponent(i, connectedComponent);
      if(canLinkNodeWithComponent(CkwsRoadmapBuilder::NODE_TO_ROADMAP,
				  node, connectedComponent,
				  CkitParameterMap::create (),
				  o_node,o_dp)
         ||
         canLinkNodeWithComponent(CkwsRoadmapBuilder::ROADMAP_TO_NODE,
				  node, connectedComponent,
				  CkitParameterMap::create (),
				  o_node,o_dp))
       {
         ncc++;
       }
    }
    if(ncc==0)
    {
      ODEBUG2(" isolated node");
      if(addNode(node)!=KD_ERROR)
      {
        link(CkwsRoadmapBuilder::NODE_TO_ROADMAP,node,
	     CkitParameterMap::create ());
        link(CkwsRoadmapBuilder::ROADMAP_TO_NODE,node,
	     CkitParameterMap::create ());
      }
      att_n_iterations=0;
    }
    else if(ncc>=2)
    {
      ODEBUG2(" node Link two cc " << ncc);
      if(addNode(node)!=KD_ERROR)
      {
        link(CkwsRoadmapBuilder::BIDIRECTIONAL,node,
	     CkitParameterMap::create ());
      }
    }
  }
}

// ==========================================================================

bool ChppVisRdmBuilder::plannerShouldStopPlanning
(const CkwsPathPlannerConstShPtr&) const
{
  ODEBUG1(" iteration "<<att_n_iterations);
  if (att_n_iterations>att_max_iterations){
    ODEBUG1(" max number of iterations reached");
    return true;
  }
  return false;
}


// ==========================================================================

double ChppVisRdmBuilder::difficulty ( ) const
{
  return 0;
}



