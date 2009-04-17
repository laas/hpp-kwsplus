/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#include "kwsPlus/roadmap/kwsPlusNode.h"
#include "kwsPlus/roadmap/kwsPlusEdge.h"

#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsSteeringMethod.h"

#include <iostream>

using namespace std;

CkwsPlusNode::~CkwsPlusNode(){
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
CkwsPlusNode::CkwsPlusNode(const CkwsConfig& inConfig):CkwsNode(inConfig){

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
CkwsPlusNodeShPtr CkwsPlusNode::create(const CkwsConfig& inConfig){

  CkwsPlusNode * ptr = new CkwsPlusNode(inConfig);
  CkwsPlusNodeShPtr shPtr(ptr);

  if(KD_ERROR == ptr->init(shPtr)){
    shPtr.reset();
  }

  return shPtr;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::init(const CkwsPlusNodeWkPtr &inWeakPtr){
  
  ktStatus success = KD_OK;
  attWeakPtr = inWeakPtr;
  attColor = CkppColor(1,1,1,1);
  attWeight = 1;
  attCollision = 0;
  attCollisionTimes = 0;
  attRetropropagation = 1;
  attInfluenceradius = 1;
  attIsActivated = true;
  attCatch = false;
  attRank = 0;
  attNbExtensionTimes = 0;
  success = CkwsNode::init(inWeakPtr);

  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeWeight(double inWeight){
  
  ktStatus success = KD_OK;
  attWeight = inWeight;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeCollisionProbability(double inCollision){
  
  ktStatus success = KD_OK;
  attCollision = inCollision;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeCollisionTimes(double inCollision){
  
  ktStatus success = KD_OK;
  attCollisionTimes = inCollision;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeColor(CkppColor inColor){
  
  ktStatus success = KD_OK;
  attColor = inColor;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeRank(int inRank){
  
  ktStatus success = KD_OK;
  attRank = inRank;

  //we will use attCatch attribute to avoid infinite recursive loops

  if(!attCatch){
    attCatch = true;
    if(countInEdges() != 0){
      
      for(unsigned int i =0; i<countInEdges(); i++){
	
	CkwsPlusNodeShPtr parent = KIT_DYNAMIC_PTR_CAST(CkwsPlusNode,inEdge(i)->startNode());
	if(!parent) return KD_ERROR;
	
	if(parent->countOutEdges() == 1){
	  parent->computeRank(getRank()+1);
	}else if(parent->getRank() > getRank()+1){
	  parent->computeRank(getRank()+1);
	}
	
      }
      
    }

    attCatch = false;

  }
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeRetroPropagationCoefficient(double inRetropropagation){
  
  ktStatus success = KD_OK;
  attRetropropagation = inRetropropagation;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeInfluenceRadius(double inInfluenceradius){
  
  ktStatus success = KD_OK;
  attInfluenceradius = inInfluenceradius;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::computeNbExtensionTimes(int inNbExtensionTimes){
  
  ktStatus success = KD_OK;
  attNbExtensionTimes = inNbExtensionTimes;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::activate(bool inState){
  //transfert of inEdge to outEdges in the case of diffusion algorithms. If there is more than one in Edge, 
  // the transfert will not happen and the function returns KD_ERROR;
  //This function is used in place of a remove Node fucntion, because there is currently no functions provided by kineoCAM
  //to do this. The advantage is that the node can be reactivated later if needed. The drawback is that it takes memory
  //even if the node is desactivated
  
  ktStatus success = KD_OK;

  if(countInEdges() == 1){//activation/desactivation works only if there is just one parent, in a diffusion algorithm case.
    if(attIsActivated == inState){//if the current state is same that the one we want to put the node in, there is nothing to do
      success = KD_OK;
    }else{ //else we must desactivate or activate edges accordingly
      attIsActivated = inState;

      CkwsEdgeShPtr inEdge = CkwsNode::inEdge(0);
      CkwsConfig startConfig(inEdge->startNode()->config());

      CkwsPlusEdgeShPtr inPlusEdge = KIT_DYNAMIC_PTR_CAST(CkwsPlusEdge, inEdge);
      if(!inPlusEdge){ cout<<"ERROR - kwsPlusNode : Edges are not of type CkwsPlusEdge."<<endl; return KD_ERROR; }
      else{ 
	inPlusEdge->activate(attIsActivated); 
	if(!attIsActivated) inEdge->startNode()->removeOutEdge(inPlusEdge); //desactivation
	else inEdge->startNode()->addOutEdge(inPlusEdge); //activation
      }

      for(unsigned int i=0; i<countOutEdges(); i++){

	CkwsEdgeShPtr outEdge = CkwsNode::outEdge(i);
	CkwsConfig endConfig(outEdge->endNode()->config());

	CkwsPlusEdgeShPtr outPlusEdge = KIT_DYNAMIC_PTR_CAST(CkwsPlusEdge, outEdge);
	if(!outPlusEdge){ cout<<"ERROR - kwsPlusNode : Edges are not of type CkwsPlusEdge."<<endl; return KD_ERROR; }
	else{ 
	  outPlusEdge->activate(attIsActivated); 
	  if(!attIsActivated){//desactivation
	    outEdge->endNode()->removeInEdge(outPlusEdge);
	    if(attCatch){
	      //make an edge beetween startConfig and endConfig;
	      CkwsDirectPathShPtr dp = roadmap()->device()->steeringMethod()->makeDirectPath(startConfig, endConfig);
	      roadmap()->addEdge(inEdge->startNode(),outEdge->endNode(),CkwsPlusEdge::create(dp));
	    }
	  }else{//activation
	    inEdge->startNode()->removeOutEdge(outEdge->endNode()->inEdge(0)); //remove the edge that link the parent of the reactivated node with the current son (outEdge->endNode()) of the reactivated node
	    outEdge->endNode()->removeInEdge(outEdge->endNode()->inEdge(0));
	    outEdge->endNode()->addInEdge(outPlusEdge);
	  }
	}	

      }

    }

  }else{
    
    cout<<"KWSPLUSNODE - ERROR - There must be only one inEdge."<<endl; //change with info function
    success = KD_ERROR;

  }

  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
ktStatus CkwsPlusNode::setCatch(bool inState){
  
  ktStatus success = KD_OK;
  attCatch = inState;
  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
