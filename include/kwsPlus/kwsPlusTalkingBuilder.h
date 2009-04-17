/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigné (LAAS-CNRS)
*/

#ifndef KWSPLUS_TALKING_BUILDER_H
#define KWSPLUS_TALKING_BUILDER_H

#warning "deprecated header file. Please include kwsPlus/roadmap/kwsPlusTalkingBuilder.h instead."

#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoUtility/kitNotificator.h"
#include "KineoWorks2/kwsNode.h"

#include <iostream>
#include <fstream>

template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusTalkingBuilder; 

/**
   \brief This shooter template class is provided to have more control over the shooting process. It can notify every shoots and allow users to save into a text file a sequence of shoot
   and to load one from a text file. It inherits from any shooter class. 
*/
template<class T >
class CkwsPlusTalkingBuilder : public T
{

 public:  

  /**
     \brief notifications
  */
  static const CkitNotification::TType   DID_BUILD_ONE_STEP;
  static const CkitNotification::TType   DID_DIFFUSE;
  static const CkitNotification::TType   DID_CHOOSE_EXTENDED_NODE;
  static const CkitNotification::TType   DID_EXTEND;

  static const std::string BUILDER_KEY;
  static const std::string EXTENDED_NODE_KEY;
  static const std::string DIFFUSED_NODE_KEY;
  static const std::string CREATED_NODE_KEY;

  /**
     \brief Destructor
   */
  ~CkwsPlusTalkingBuilder();

  /**
     \brief Create Method.
     \param inRoadmap the kwsRoadmap to use for the builder
     \return A shared pointer on the newly created object.
   */
  static KIT_SHARED_PTR(CkwsPlusTalkingBuilder<T>) create(const CkwsRoadmapShPtr & inRoadmap);

 protected:

  /**
     \brief Initialisation method.
     \param inWeakPtr Weak pointer on the object itself.
     \return KD_OK | KD_ERROR
   */
  ktStatus init(const KIT_WEAK_PTR(CkwsPlusTalkingBuilder<T>) &inWeakPtr);

  /**
     \brief Constructor
   */
  CkwsPlusTalkingBuilder(const CkwsRoadmapShPtr& i_roadmap);

  /**
     \brief Intherithed methods
     @{
   */
  virtual ktStatus buildOneStep ();
  virtual CkwsNodeShPtr diffuse(const CkwsNodeShPtr &i_node, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type, CkwsRoadmapBuilder::EDirection &o_direction);
  virtual CkwsNodeShPtr chooseExtendedNode(const CkwsConfig &i_cfg, CkwsConnectedComponent::TNodeConstLinkedIterator &i_beginNodeIterator, CkwsConnectedComponent::TNodeConstLinkedIterator &i_endNodeIterator);
  virtual CkwsNodeShPtr extend(const CkwsNodeShPtr &i_node, const CkwsConfig &i_cfg, CkwsRoadmapBuilder::EDirection i_direction);
  /**
     @}
   */

 private:
  KIT_WEAK_PTR(CkwsPlusTalkingBuilder<T>) m_weakPtr;

};

template <class T>
const CkitNotification::TType CkwsPlusTalkingBuilder<T>::DID_BUILD_ONE_STEP(CkitNotification::makeID());
template <class T>
const CkitNotification::TType CkwsPlusTalkingBuilder<T>::DID_DIFFUSE(CkitNotification::makeID());
template <class T>
const CkitNotification::TType CkwsPlusTalkingBuilder<T>::DID_CHOOSE_EXTENDED_NODE(CkitNotification::makeID());
template <class T>
const CkitNotification::TType CkwsPlusTalkingBuilder<T>::DID_EXTEND(CkitNotification::makeID());


template <class T>
const std::string CkwsPlusTalkingBuilder<T>::BUILDER_KEY("Builder");
template <class T>
const std::string CkwsPlusTalkingBuilder<T>::EXTENDED_NODE_KEY("ExtendedNode");
template <class T>
const std::string CkwsPlusTalkingBuilder<T>::DIFFUSED_NODE_KEY("DiffusedNode");
template <class T>
const std::string CkwsPlusTalkingBuilder<T>::CREATED_NODE_KEY("CreatedNode");

/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
CkwsPlusTalkingBuilder<T>::CkwsPlusTalkingBuilder(const CkwsRoadmapShPtr& i_roadmap):T(i_roadmap){
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
CkwsPlusTalkingBuilder<T>::~CkwsPlusTalkingBuilder(){
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
KIT_SHARED_PTR(CkwsPlusTalkingBuilder<T>) CkwsPlusTalkingBuilder<T>::create(const CkwsRoadmapShPtr &i_roadmap){

  CkwsPlusTalkingBuilder<T>* ptr = new CkwsPlusTalkingBuilder(i_roadmap);
  KIT_SHARED_PTR(CkwsPlusTalkingBuilder<T>) shPtr(ptr);

  if(KD_ERROR == ptr->init(shPtr)){

    shPtr.reset();

  }

  return shPtr;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
ktStatus CkwsPlusTalkingBuilder<T>::init(const KIT_WEAK_PTR(CkwsPlusTalkingBuilder<T>) &i_weakPtr){

  m_weakPtr = i_weakPtr;

  return T::init(i_weakPtr);
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
ktStatus CkwsPlusTalkingBuilder<T>::buildOneStep(){
    
  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsPlusTalkingBuilder<T> >(CkwsPlusTalkingBuilder<T>::DID_BUILD_ONE_STEP, m_weakPtr.lock());
//  notification->shPtrValue<CkwsDiffusingRdmBuilder>(BUILDER_KEY, m_weak);
  CkitNotificator::defaultNotificator()->notify(notification);

  return T::buildOneStep();

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
CkwsNodeShPtr CkwsPlusTalkingBuilder<T>::diffuse(const CkwsNodeShPtr &i_node, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type,  CkwsRoadmapBuilder::EDirection &o_direction){
    
  CkwsNodeShPtr newNodeShPtr = T::diffuse(i_node,i_type,o_direction);

  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsPlusTalkingBuilder<T> >(CkwsPlusTalkingBuilder<T>::DID_DIFFUSE, m_weakPtr.lock());
  notification->constShPtrValue<CkwsNode>(DIFFUSED_NODE_KEY, newNodeShPtr);
  CkitNotificator::defaultNotificator()->notify(notification);

  return newNodeShPtr;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
CkwsNodeShPtr CkwsPlusTalkingBuilder<T>::chooseExtendedNode(const CkwsConfig &i_cfg, CkwsConnectedComponent::TNodeConstLinkedIterator &i_beginNodeIterator, CkwsConnectedComponent::TNodeConstLinkedIterator &i_endNodeIterator){ 
    
  CkwsNodeShPtr newNode = T::chooseExtendedNode(i_cfg,i_beginNodeIterator,i_endNodeIterator);
    
  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsPlusTalkingBuilder<T> >(CkwsPlusTalkingBuilder<T>::DID_CHOOSE_EXTENDED_NODE, m_weakPtr.lock());
  notification->constShPtrValue<CkwsNode>(EXTENDED_NODE_KEY, newNode);
  CkitNotificator::defaultNotificator()->notify(notification);

  return newNode;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
CkwsNodeShPtr CkwsPlusTalkingBuilder<T>::extend(const CkwsNodeShPtr &i_node, const CkwsConfig &i_cfg,  CkwsRoadmapBuilder::EDirection i_direction){
    
  CkwsNodeShPtr newNode = T::extend(i_node,i_cfg,i_direction);
    
  CkitNotificationShPtr notification = CkitNotification::createWithShPtr<CkwsPlusTalkingBuilder<T> >(CkwsPlusTalkingBuilder<T>::DID_EXTEND, m_weakPtr.lock());
  notification->constShPtrValue<CkwsNode>(CREATED_NODE_KEY, newNode);
  CkitNotificator::defaultNotificator()->notify(notification);

  return newNode;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


#endif
