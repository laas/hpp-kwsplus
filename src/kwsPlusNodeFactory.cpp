/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#include "kwsPlusNodeFactory.h"

#include <iostream>


CkwsPlusNodeFactory::CkwsPlusNodeFactory(){
}
CkwsPlusNodeFactory::~CkwsPlusNodeFactory(){
  cout<<"KWSPLUS NODE FACTORY  - Deleting Object"<<endl;
}
CkwsPlusNodeFactoryShPtr CkwsPlusNodeFactory::create(){
  cout<<"KWSPLUS NODE FACTORY  - CREATING Object"<<endl;

  CkwsPlusNodeFactory * ptr = new CkwsPlusNodeFactory();
  CkwsPlusNodeFactoryShPtr shPtr(ptr);

  if(KD_ERROR == ptr->init(shPtr)){

    shPtr.reset();

  }

  return shPtr;

}
ktStatus CkwsPlusNodeFactory::init(const CkwsPlusNodeFactoryWkPtr &inWeakPtr){

  return CkwsNodeFactory::init(inWeakPtr);

}
CkwsNodeShPtr CkwsPlusNodeFactory::makeNode(const CkwsConfig &inCfg) const{

  CkwsPlusNodeShPtr newNode = CkwsPlusNode::create(inCfg);
  return newNode;

}
