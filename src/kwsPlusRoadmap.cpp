/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne

*/

#include <iostream>

#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoController/kppInsertComponentCommand.h"
#include "KineoController/kppInsertSolidComponentCommand.h"
#include "KineoModel/kppModelTree.h"
#include "KineoModel/kppGeometryNode.h"
#include "KineoModel/kppDeviceComponent.h"
#include "KineoModel/kppJointComponent.h"
#include "KineoWorks2/kwsJoint.h"

#include "kwsPlusRoadmap.h"

using namespace std;

CkwsPlusRoadmap::CkwsPlusRoadmap(){
}

CkwsPlusRoadmap::~CkwsPlusRoadmap(){
}

void CkwsPlusRoadmap::init(const CkwsDeviceShPtr & i_device, const CkwsPlusRoadmapWkPtr& inRoadmapWkPtr ,const std::string &i_name){

  if( CkppKCDAssembly::init(inRoadmapWkPtr,inRoadmapWkPtr, i_name) == KD_ERROR ) cout<<"Failed to init KCDAssembly"<<endl;
  CkwsRoadmap::init(i_device,inRoadmapWkPtr);
  activation(false);
  isInModelTree = false;

}

CkwsPlusRoadmapShPtr CkwsPlusRoadmap::create(const CkwsDeviceShPtr & i_device,const std::string &inName){

  CkwsPlusRoadmap * roadmapPtr = new CkwsPlusRoadmap();
  CkwsPlusRoadmapShPtr inRoadmap(roadmapPtr);

  inRoadmap->init(i_device,inRoadmap,inName);
  
  return inRoadmap;

}

void CkwsPlusRoadmap::compute(){

  unsigned int point_rank[6] ;
  double scale = 0.05;

  CkppKCDPolyhedronShPtr rdmEdges = CkppKCDPolyhedron::create("edges");
  
  //
  // Retrieve vector of joints that should be displayed.
  //
  CkwsDeviceShPtr rdmDevice = device();
  CkwsDevice::TJointVector jointVector;
  rdmDevice->getJointVector(jointVector);
  std::vector<CkwsJointShPtr> displayJointVector;
  for (unsigned int iJoint=0; iJoint<jointVector.size(); iJoint++) {
    CkppJointComponentShPtr kppJoint = KIT_DYNAMIC_PTR_CAST(CkppJointComponent, jointVector[iJoint]);
    if (kppJoint) {
      if (kppJoint->doesDisplayPath()) {
	displayJointVector.push_back(jointVector[iJoint]);
      }
    }
  }
  if (displayJointVector.size() == 0) {
    std::cout << "CkwsPlusRoadmap::compute: no joint to display." << std::endl;
    return;
  }

  if(!countNodes()) cout<<"No nodes in the roadmap"<<endl;
  else{
    cout<<"displaying roadmap..."<<endl;
    for (unsigned int iJoint=0; iJoint < displayJointVector.size(); iJoint++) {
      for(int i=0; i<countNodes(); i++){//pour chaque noeud de la roadmap
	for(int j=0; j<node(i)->countOutEdges();j++){//Pour chaque arc sortant du noeud
	  CkwsJointShPtr kwsJoint = displayJointVector[iJoint];
	  CkwsConfig current(node(i)->config());//current configuration : edge start
	  CkwsConfig next(node(i)->outEdge(j)->endNode()->config());//next configuration : edge end
	
	  rdmDevice->setCurrentConfig(current);
	  CkitMat4 jointPosition = kwsJoint->currentPosition();
	  double x1 = jointPosition(0,3);
	  double y1 = jointPosition(1,3);
	  double z1 = jointPosition(2,3);

	  rdmDevice->setCurrentConfig(next);
	  jointPosition = kwsJoint->currentPosition();
	  double x2 = jointPosition(0,3);
	  double y2 = jointPosition(1,3);
	  double z2 = jointPosition(2,3);

	  //creating triangles to design an edge (toblerone-like)
	  rdmEdges->CkcdPolyhedron::addPoint(x1-scale , y1  , z1 , point_rank[0]);
	  rdmEdges->CkcdPolyhedron::addPoint(x1 , y1-scale , z1 , point_rank[1]);
	  rdmEdges->CkcdPolyhedron::addPoint(x1 , y1 , z1-scale , point_rank[2]);
	  
	  rdmEdges->CkcdPolyhedron::addPoint(x2-scale , y2 , z2 , point_rank[3]);
	  rdmEdges->CkcdPolyhedron::addPoint(x2 , y2-scale , z2 , point_rank[4]);
	  rdmEdges->CkcdPolyhedron::addPoint(x2 , y2 , z2-scale , point_rank[5]);
	  
	  rdmEdges->addTriangle(point_rank[0] , point_rank[3] , point_rank[1]);
	  rdmEdges->addTriangle(point_rank[1] , point_rank[3] , point_rank[4]);
	  rdmEdges->addTriangle(point_rank[1] , point_rank[4] , point_rank[5]);
	  rdmEdges->addTriangle(point_rank[1] , point_rank[5] , point_rank[2]);
	  rdmEdges->addTriangle(point_rank[2] , point_rank[5] , point_rank[3]);
	  rdmEdges->addTriangle(point_rank[2] , point_rank[3] , point_rank[0]);
	}
       }
    }
    insertChildComponent(rdmEdges,0);
  }
  

}

void CkwsPlusRoadmap::display(){
    
  CkppDeviceComponentShPtr robot = KIT_DYNAMIC_PTR_CAST(CkppDeviceComponent, this->device());
  CkppModelTreeShPtr modelTree = robot->modelTree();

  if (modelTree) {
    CkppInsertComponentCommandShPtr insertCommand;
    
    insertCommand = CkppInsertSolidComponentCommand::create();
    insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::PARENT_COMPONENT), 
			      CkppComponentShPtr(modelTree->geometryNode()));
    
    
    insertCommand->paramValue(insertCommand->parameter(CkppInsertComponentCommand::INSERTED_COMPONENT), 
			      CkppComponentShPtr(this));
    insertCommand->doExecute() ;
  }
}

void CkwsPlusRoadmap::lastEdge(){

  unsigned int point_rank[6] ;
  double scale = 0.05;

  CkppKCDPolyhedronShPtr newEdge = CkppKCDPolyhedron::create("edge");

  //
  // Retrieve vector of joints that should be displayed.
  //
  CkwsDeviceShPtr rdmDevice = device();
  CkwsDevice::TJointVector jointVector;
  rdmDevice->getJointVector(jointVector);
  std::vector<CkwsJointShPtr> displayJointVector;
  for (unsigned int iJoint=0; iJoint<jointVector.size(); iJoint++) {
    CkppJointComponentShPtr kppJoint = KIT_DYNAMIC_PTR_CAST(CkppJointComponent, jointVector[iJoint]);
    if (kppJoint) {
      if (kppJoint->doesDisplayPath()) {
	displayJointVector.push_back(jointVector[iJoint]);
      }
    }
  }
  if (displayJointVector.size() == 0) {
    std::cout << "CkwsPlusRoadmap::compute: no joint to display." << std::endl;
    return;
  }

  if(countNodes()){
    
    if(node(countNodes()-1)->countOutEdges()){//adding only the edge corresponding to the last node
      for(int j=0; j<node(countNodes()-1)->countOutEdges();j++){//Pour chaque arc sortant du noeud
	for (unsigned int iJoint=0; iJoint < displayJointVector.size(); iJoint++) {
  
	  CkwsConfig current(node(countNodes()-1)->config());//current configuration : edge start
	  CkwsConfig next(node(countNodes()-1)->outEdge(j)->endNode()->config());//next configuration : edge end
	  
	  rdmDevice->setCurrentConfig(current);
	  CkwsJointShPtr kwsJoint = displayJointVector[iJoint];
	  CkitMat4 jointPosition = kwsJoint->currentPosition();
	  double x1 = jointPosition(0,3);
	  double y1 = jointPosition(1,3);
	  double z1 = jointPosition(2,3);

	  rdmDevice->setCurrentConfig(next);
	  jointPosition = kwsJoint->currentPosition();
	  double x2 = jointPosition(0,3);
	  double y2 = jointPosition(1,3);
	  double z2 = jointPosition(2,3);

	  newEdge->CkcdPolyhedron::addPoint(x1-scale , y1  , z1 , point_rank[0]);
	  newEdge->CkcdPolyhedron::addPoint(x1 , y1-scale , z1 , point_rank[1]);
	  newEdge->CkcdPolyhedron::addPoint(x1 , y1 , z1-scale , point_rank[2]);
	  
	  newEdge->CkcdPolyhedron::addPoint(x2-scale , y2 , z2 , point_rank[3]);
	  newEdge->CkcdPolyhedron::addPoint(x2 , y2-scale , z2 , point_rank[4]);
	  newEdge->CkcdPolyhedron::addPoint(x2 , y2 , z2-scale , point_rank[5]);
	  
	  newEdge->addTriangle(point_rank[0] , point_rank[3] , point_rank[1]);
	  newEdge->addTriangle(point_rank[1] , point_rank[3] , point_rank[4]);
	  newEdge->addTriangle(point_rank[1] , point_rank[4] , point_rank[5]);
	  newEdge->addTriangle(point_rank[1] , point_rank[5] , point_rank[2]);
	  newEdge->addTriangle(point_rank[2] , point_rank[5] , point_rank[3]);
	  newEdge->addTriangle(point_rank[2] , point_rank[3] , point_rank[0]);
	}
      }
      insertChildComponent(newEdge,0);
    }
  }
  else{ cout<<"ERROR : No Nodes in the roadmap"<<endl;}
}
