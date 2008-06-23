/*
   Copyright (c) JRL CNRS-AIST,
   @author : Florent Lamiraux

*/

#include "kwsPlusStopRdmBuilderDelegate.h"

#if DEBUG==2
unsigned int CkwsPlusStopRdmBuilderDelegate::attNbInstances = 0;
#endif

CkwsPlusStopRdmBuilderDelegate::CkwsPlusStopRdmBuilderDelegate() : 
  attShouldStop(false) 
{
#if DEBUG==2   
  attNbInstances++;
  std::cout << "CkwsPlusStopRdmBuilderDelegate: nb instances = " << attNbInstances << std::endl;
#endif
}

CkwsPlusStopRdmBuilderDelegate::~CkwsPlusStopRdmBuilderDelegate()
{
#if DEBUG==2
  attNbInstances--;
  std::cout << "CkwsPlusStopRdmBuilderDelegate: nb instances = " << attNbInstances << std::endl;
#endif
}
