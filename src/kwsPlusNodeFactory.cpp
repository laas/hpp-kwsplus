/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#include "kwsPlus/roadmap/kwsPlusNodeFactory.h"

#include <iostream>

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CkwsPlusNodeFactory:" << x << std::endl
#define ODEBUG2(x) std::cout << "CkwsPlusNodeFactory:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusNodeFactory:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CkwsPlusNodeFactory:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusNodeFactory:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CkwsPlusNodeFactory:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

CkwsPlusNodeFactory::CkwsPlusNodeFactory(){
}
CkwsPlusNodeFactory::~CkwsPlusNodeFactory(){
  ODEBUG2("KWSPLUS NODE FACTORY  - Deleting Object");
}
CkwsPlusNodeFactoryShPtr CkwsPlusNodeFactory::create(){
  ODEBUG2("KWSPLUS NODE FACTORY  - CREATING Object");

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
