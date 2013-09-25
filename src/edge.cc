/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#include <hpp/kwsplus/roadmap/edge.hh>

CkwsPlusEdge::~CkwsPlusEdge(){
}
CkwsPlusEdge::CkwsPlusEdge(){
}
CkwsPlusEdgeShPtr CkwsPlusEdge::create(const CkwsDirectPathConstShPtr &inDirectPath){

  CkwsPlusEdge * ptr = new CkwsPlusEdge();
  CkwsPlusEdgeShPtr shPtr(ptr);

  if(KD_ERROR == ptr->init(inDirectPath,shPtr)){

    shPtr.reset();

  }

  return shPtr;

}
ktStatus CkwsPlusEdge::init(const CkwsDirectPathConstShPtr &inDirectPath,const CkwsPlusEdgeWkPtr &inWeakPtr){

  attIsActivated = true;
  attColor = CkppColor(1,1,1,1);
  return CkwsEdge::init(inDirectPath,inWeakPtr);

}
ktStatus CkwsPlusEdge::activate(bool inState){

  attIsActivated = inState;

  return KD_OK;

}
bool CkwsPlusEdge::isEdgeActivated(){

  return attIsActivated;

}
