/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#include <hpp/kwsplus/roadmap/edge-factory.hh>

CkwsPlusEdgeFactory::CkwsPlusEdgeFactory(){
}
CkwsPlusEdgeFactory::~CkwsPlusEdgeFactory(){
}
CkwsPlusEdgeFactoryShPtr CkwsPlusEdgeFactory::create(){

  CkwsPlusEdgeFactory * ptr = new CkwsPlusEdgeFactory();
  CkwsPlusEdgeFactoryShPtr shPtr(ptr);

  if(KD_ERROR == ptr->init(shPtr)){

    shPtr.reset();

  }

  return shPtr;

}
ktStatus CkwsPlusEdgeFactory::init(const CkwsPlusEdgeFactoryWkPtr&){

  return KD_OK;

}
CkwsEdgeShPtr CkwsPlusEdgeFactory::makeEdge(const CkwsDirectPathConstShPtr &inDirectPath) const{

  CkwsPlusEdgeShPtr newEdge = CkwsPlusEdge::create(inDirectPath);

  return newEdge;

}
