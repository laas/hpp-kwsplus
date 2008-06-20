/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigné (LAAS-CNRS)
*/

#ifndef KWSPLUS_TALKING_SHOOTER_H
#define KWSPLUS_TALKING_SHOOTER_H

#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoUtility/kitNotificator.h"

#include <iostream>
#include <fstream>

template<class T = CkwsDiffusionShooter > class CkwsPlusTalkingShooter; 

using namespace std;
/**
   \brief This shooter template class is provided to have more control over the shooting process. It can notify every shoots and allow users to save into a text file a sequence of shoot
   and to load one from a text file. It inherits from any shooter class. 
*/
template<class T >
class CkwsPlusTalkingShooter : public T
{

 public:  

  /**
     \brief notifications
  */
  static const CkitNotification::TType   DID_SHOOT;
  static const CkitNotification::TType   DID_GAUSSIAN_SHOOT;
  static const CkitNotification::TType   DID_FREE_SHOOT;

  static const std::string CONFIG_KEY;
  static const std::string DIFFUSION_NODE_KEY;

  static KIT_SHARED_PTR(CkwsPlusTalkingShooter<T>) create ();

  /**
     \brief Destructor
   */
  virtual ~CkwsPlusTalkingShooter ();
  virtual void reset();

  /**
     \brief Shoots a configuration according to the mother's class shoot function, after sending a notification to the default notificator.
     if willPlayShoots has been set to true, it read a CkwsConfig from the shoot vector instead of doing a real shoot.
     \return KD_OK if a configuration has been successfully shot, KD_ERROR otherwise.
  */
  virtual ktStatus shoot(const CkwsNodeShPtr &i_node, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type, CkwsConfig &o_cfg);
  virtual void lastShootWasUseful(const CkwsNodeConstShPtr &i_extendedNode, const CkwsNodeConstShPtr &i_newNode, CkwsRoadmapBuilder::EDirection i_direction);
  virtual void lastShootWasUseless(const CkwsNodeConstShPtr &i_extendedNode, CkwsRoadmapBuilder::EDirection i_direction);

  /**
     \brief read a text file to extract configuration from it and put it in the internal shoot vector
     \param filename name of the file to read
     \return KD_OK|KD_ERROR
  */
  ktStatus openShootFile(std::string filename);

  /**
     \brief Saves every configuration given by the base class shoot function in the internal vector of configurations
     \param filename name of the file where configurations are saved
     \return KD_OK|KD_ERROR
   */
  ktStatus saveShootFile(std::string filename);

  /**
     \brief Set the "saveShoots" attribute, saying if the shoots are stored in the internal vector.
     \param saving if true, shoots are stored in the internal vector
   */
  void areShootsSaved(bool saving);
  /**
     \brief Get the state of "saveShoots" attribute (false or true)
   */
  bool areShootsSaved();

  /**
     \brief Set the playShoots attribute, saying if the future shoots will be really chosen at random or only picked from the internal vector
     \param play if true, shoots will be picked from the internal vector of configurations
   */
  void willPlayShoots(bool play);
  /**
     \brief Get the state of the "playShoots" attribute.
   */
  bool willPlayShoots();

 protected :

  /**
     \brief Constructor
   */
  CkwsPlusTalkingShooter();
  ktStatus init(const KIT_WEAK_PTR(CkwsPlusTalkingShooter<T>) &i_weakPtr);
  virtual ktStatus doGaussianShoot(std::vector< double > &io_dofValues, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type) const;

 private:
  double nbShoots;
  double nbCollidingShoots;
  int rank; //rank of the shoot in the shoot vector, when shoots are picked from this vector.
  bool saveShoots;
  bool playShoots;
  vector<CkwsConfig> shootVector;
  std::string m_filename;
  KIT_WEAK_PTR(CkwsPlusTalkingShooter<T>) m_weakPtr;

};

template <class T>
const CkitNotification::TType CkwsPlusTalkingShooter<T>::DID_SHOOT(CkitNotification::makeID());
template <class T>
const CkitNotification::TType CkwsPlusTalkingShooter<T>::DID_GAUSSIAN_SHOOT(CkitNotification::makeID());
template <class T>
const CkitNotification::TType CkwsPlusTalkingShooter<T>::DID_FREE_SHOOT(CkitNotification::makeID());

template <class T>
const std::string CkwsPlusTalkingShooter<T>::CONFIG_KEY("configuration");
template <class T>
const std::string CkwsPlusTalkingShooter<T>::DIFFUSION_NODE_KEY("diffusion_node");

/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

template <class T>
CkwsPlusTalkingShooter<T>::CkwsPlusTalkingShooter(){
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
CkwsPlusTalkingShooter<T>::~CkwsPlusTalkingShooter (){
  m_weakPtr.reset();
  shootVector.clear();
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
KIT_SHARED_PTR(CkwsPlusTalkingShooter<T>) CkwsPlusTalkingShooter<T>::create(){

  CkwsPlusTalkingShooter<T>*  ptr = new CkwsPlusTalkingShooter();
  KIT_SHARED_PTR(CkwsPlusTalkingShooter<T>) shPtr(ptr);
  
  if(KD_OK != ptr->init(shPtr)){
    shPtr.reset();
  }

  return shPtr;
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
void CkwsPlusTalkingShooter<T>::reset(){

  nbShoots = 0;
  nbCollidingShoots = 0;
  rank = 0;
  saveShoots = false;
  playShoots = false;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

template <class T>
ktStatus CkwsPlusTalkingShooter<T>::init(const KIT_WEAK_PTR(CkwsPlusTalkingShooter<T>) &i_weakPtr){

  nbShoots = 0;
  nbCollidingShoots = 0;
  rank = 0;
  saveShoots = false;
  playShoots = false;
  m_weakPtr = i_weakPtr;

  return KD_OK;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
ktStatus CkwsPlusTalkingShooter<T>::doGaussianShoot(std::vector< double > &io_dofValues, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type) const{

  ktStatus success;
  success = T::doGaussianShoot(io_dofValues,i_type);

  if(success == KD_OK){
    CkwsConfig * newConfig = new CkwsConfig(T::builder()->roadmap()->device(),io_dofValues);
    
    CkitNotificationShPtr notification = CkitNotification::createWithPtr<CkwsPlusTalkingShooter<T> >(CkwsPlusTalkingShooter<T>::DID_GAUSSIAN_SHOOT, m_weakPtr.lock().get());
    notification->ptrValue<CkwsConfig>(CONFIG_KEY, newConfig);
    CkitNotificator::defaultNotificator()->notify(notification);
  }

  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
ktStatus CkwsPlusTalkingShooter<T>::shoot(const CkwsNodeShPtr &i_node, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type, CkwsConfig &o_cfg){

  ktStatus success;
  bool collision;

  if(!playShoots){

    success = T::shoot(i_node,i_type,o_cfg);

    if(success == KD_OK){
      
      CkitNotificationShPtr notification = CkitNotification::createWithPtr<CkwsPlusTalkingShooter<T> >(CkwsPlusTalkingShooter<T>::DID_SHOOT, m_weakPtr.lock().get());
      notification->ptrValue<CkwsConfig>(CONFIG_KEY, &o_cfg);
      CkitNotificator::defaultNotificator()->notify(notification);    
      nbShoots++;
      
      if(o_cfg.isColliding(collision)){
	if(collision) 
	  nbCollidingShoots++; 
      }
      
      if(saveShoots){
	shootVector.push_back(o_cfg);
      }
      
    }
  }
  else{

    if(rank < shootVector.size())
    o_cfg = shootVector[rank++];

  }

  return success;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
void CkwsPlusTalkingShooter<T>::lastShootWasUseful(const CkwsNodeConstShPtr &i_extendedNode, const CkwsNodeConstShPtr &i_newNode, CkwsRoadmapBuilder::EDirection i_direction){
  T::lastShootWasUseful(i_extendedNode,i_newNode,i_direction);
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
void CkwsPlusTalkingShooter<T>::lastShootWasUseless(const CkwsNodeConstShPtr &i_extendedNode, CkwsRoadmapBuilder::EDirection i_direction){
  T::lastShootWasUseless(i_extendedNode,i_direction);
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
ktStatus CkwsPlusTalkingShooter<T>::openShootFile(std::string filename){

  m_filename = filename;
  double temp;

  fstream filestr(m_filename, fstream::in);

  while(!filestr.eof()){

    std::vector< double > &i_dofValues;

    for(int i = 0; i<T::builder()->roadmap()->device()->countDofs();i++ ){

      filestr>>temp;
      if(filestr.good()) i_dofValues.push_back(temp);
      else cout<<"ERROR - cannot retrieve a config from file"<<endl;

    }

    shootVector.push_back(CkwsConfig(T::builder()->roadmap()->device(),i_dofValues));

  }

  filestr.close();

  return KD_OK;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
ktStatus CkwsPlusTalkingShooter<T>::saveShootFile(std::string filename){

  m_filename = filename;

  if(shootVector.size() == 0){
    cout<<"Cannot save shoots because Shoots Vector is empty."<<endl;
    return KD_ERROR;
  }

  fstream filestr(m_filename, fstream::out | fstream::app);
  std::vector< double > &o_dofValues;
  for(vector<CkwsConfig>::iterator It = shootVector.begin(); It != shootVector.end();It++){
    if(KD_ERROR == (*It).getDofValues(o_dofValues)){cout<<"ERROR - Cannot retrieve dof values from shooted configuration"; return KD_ERROR;}
    
    for(vector<double>::iterator It2 = o_dofValues.begin(); It2 != o_dofValues.end(); It2++ ){
      filestr<<*It2<<" ";
    }
    filestr<<endl;

  }

  filestr.close();

  return KD_OK;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
void CkwsPlusTalkingShooter<T>::areShootsSaved(bool saving){
  
  saveShoots = saving;
  
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
bool CkwsPlusTalkingShooter<T>::areShootsSaved(){

  return saveShoots;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
void CkwsPlusTalkingShooter<T>::willPlayShoots(bool play){

  playShoots = play;

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
template <class T>
bool CkwsPlusTalkingShooter<T>::willPlayShoots(){

  return playShoots;

}
#endif
