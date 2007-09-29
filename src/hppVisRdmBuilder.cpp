
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

#include "hppVisRdmBuilder.h"
#include "hppShooterActiveDof.h"


// ==========================================================================

ChppVisRdmBuilderShPtr ChppVisRdmBuilder::create(const CkwsRoadmapShPtr &i_roadmap,
                                                 double i_penetration,
                                                 const ChppShooterActiveDofShPtr &i_shooter,
                                                 const CkwsDistanceShPtr &i_evaluator)
{

  ChppVisRdmBuilder* devPtr = new ChppVisRdmBuilder(i_roadmap,i_evaluator);
  ChppVisRdmBuilderShPtr devShPtr(devPtr);
  ChppVisRdmBuilderWkPtr devWkPtr(devShPtr);

  if (devPtr->init(devWkPtr,i_penetration,i_shooter) != KD_OK){
    devShPtr.reset();
  }

  return devShPtr;

}

// ==========================================================================

ChppVisRdmBuilder::ChppVisRdmBuilder(const CkwsRoadmapShPtr& i_roadmap,
                                     const CkwsDistanceShPtr& i_evaluator)
  : CkwsRoadmapBuilder(i_roadmap,i_evaluator)
{}

// ==========================================================================

ktStatus ChppVisRdmBuilder::init(const ChppVisRdmBuilderWkPtr& i_weakPtr, double i_penetration,
                                 const ChppShooterActiveDofShPtr &i_shooter)
{
  ktStatus success = CkwsRoadmapBuilder::init(i_weakPtr,i_penetration);
  att_n_iterations = 0;
  att_shooter=i_shooter;

  this->roadmap()->device()->userDirectPathValidators()->add(CkwsValidatorDPCollision::create(this->roadmap()->device(),0.0001));
  this->roadmap()->device()->userPathValidators()->add(CkwsValidatorPathCollision::create(this->roadmap()->device(),0.0001));
  this->roadmap()->device()->userConfigValidators()->add(CkwsValidatorCfgCollision::create (this->roadmap()->device()));


  return success ;
}


// ==========================================================================

ktStatus ChppVisRdmBuilder::buildOneStep()
{
  CkwsConfig currentConf(this->roadmap()->device());
  CkwsConfig config(this->roadmap()->device());
  this->roadmap()->device()->getCurrentConfig(currentConf);
  CkwsNodeShPtr currentNode = CkwsNode::create(currentConf);


  std::vector<double> dofs;
  currentConf.getDofValues(dofs);

  CkwsConstraintSetShPtr constraints = this->roadmap()->device()->userConstraints();
  constraints->isActivated(false);

  if(att_shooter->shoot(currentNode,config))
  {
    constraints->isActivated(true);
    constraints->apply(config);
    bool val=true;
    CkwsValidatorCfgCollisionShPtr cfv = CkwsValidatorCfgCollision::create (this->roadmap()->device());
    CkwsNodeShPtr node = CkwsNode::create(config);
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
  return KD_OK;
}


// ==========================================================================

void ChppVisRdmBuilder::addVisibilityNode(CkwsNodeShPtr node)
{
  this->nbNearestNodes(10);
  if(this->roadmap()->countNodes()<2)
  {
    if(this->roadmap()->addNode(node)!=KD_ERROR)
    {
      this->link(CkwsRoadmapBuilder::BIDIRECTIONAL,node);
    }
  }
  else
  {
    int ncc=0;

    CkwsNodeShPtr o_node = CkwsNode::create(node->config());
    CkwsDirectPathShPtr o_dp;
    std::cout<<"Nbr cc "<<this->roadmap()->countConnectedComponents()<<std::endl;

    for(unsigned int i=0; i<this->roadmap()->countConnectedComponents(); i++)
    {
      if(this->canLinkNodeWithComponent(CkwsRoadmapBuilder::NODE_TO_ROADMAP,
                                       node,this->roadmap()->connectedComponent(i),
                                       o_node,o_dp)
         ||
         this->canLinkNodeWithComponent(CkwsRoadmapBuilder::ROADMAP_TO_NODE,
                                       node,this->roadmap()->connectedComponent(i),
                                       o_node,o_dp))
       {
         ncc++;
       }
    }
    if(ncc==0)
    {
      std::cout<<"Isolated node"<<std::endl;
      if(this->roadmap()->addNode(node)!=KD_ERROR)
      {
        this->link(CkwsRoadmapBuilder::NODE_TO_ROADMAP,node);
        this->link(CkwsRoadmapBuilder::ROADMAP_TO_NODE,node);
      }
      att_n_iterations=0;
    }
    else if(ncc>=2)
    {
      std::cout<<"Node Link two cc "<<ncc<<std::endl;
      if(this->roadmap()->addNode(node)!=KD_ERROR)
      {

        this->link(CkwsRoadmapBuilder::BIDIRECTIONAL,node);

      }
    }
  }
}

// ==========================================================================

bool ChppVisRdmBuilder::shouldStopBuilding ( ) const
{
  if (att_n_iterations>2000){
    std::cout << "max number of iterations reached" << std::endl;
    return true;
  }
  return false;
}


// ==========================================================================

double ChppVisRdmBuilder::difficulty ( ) const
{
  return 0;
}



