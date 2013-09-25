/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigné (LAAS-CNRS)
*/


#ifndef KWS_PLUS_LT_ROADMAP_BUILDER
#define KWS_PLUS_LT_ROADMAP_BUILDER

/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsValidatorDPCollision.h"
#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsShooterConfigSpace.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRoadmap.h"
#include <KineoWorks2/kwsMetricEuclidean.h>
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDiffusionNodePicker.h"
#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoWorks2/kwsPickerBasic.h"
#include "KineoWorks2/kwsShooterRoadmapBox.h"
#include "KineoWorks2/kwsShooterConfigSpace.h"
#include "KineoWorks2/kwsConnectedComponent.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsDof.h"
#include "KineoWorks2/kwsEdge.h"

#include "KineoUtility/kitInterface.h"
#include "KineoWorks2/kwsDefine.h"

#include <vector>


KIT_PREDEF_CLASS( CkwsRoadmap );


template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusLTRdmBuilder; 

class CkwsDiffusingRdmBuilder;
class CkwsIPPRdmBuilder;

/*************************************
CLASS
**************************************/

/**
   \addtogroup LT
   @{
   This part implements the local trees algorithm presented by Molten Strandberg in "Augmenting RRT-Planners with Local Trees"
   \section loctree Local Trees Algorithm
   The principle of Local Trees is based on the idea of bi-diffuse algorithms. For diffusion algorithms, a common implementation is
   to grow trees from the start configuration AND from the goal, in order to increase success probability. Local Trees purpose is creating
   new trees in areas that can't be reached directly from the start tree nor the goal tree. The result is that short narrow passages are
   found easier (Short narrow passages means doors between two large empty rooms and not corridors).
   
   \image html LocalTreesProblem.png "Typical Problem for which Local Trees Algorithm is well-suited" 

   \section useLT Using Local Trees in Diffusing Algorithms
   The Local Trees method works well in problems where there large empty rooms separated by short narrow passages. For these, the classical Local Trees algorithm
   (means Local Trees executed with Classical RRT as the template class : CkwsDiffusingRdmBuilder). Executed with the Iterative Path Planner (IPP) method, it can
   overcome the problem of corridors and constrained environements.
   Local Trees method can also be executed with the PCA, but to take advantages of both algorithms one must do the template process like this:
   - Give Local Trees Roadmap builder as the template class for PCA.
   - Give any diffusing algorithm as template class for Local Trees.

   This order (PCA -> LT -> any Diffusing) is due to the specific implementation of both classes.
   

*/
/**
   \brief This template class inherits from any diffusing roadmap builder class,
   and it augment diffusing algorithms by creating local trees in areas that are
   unreachable by the current roadmap.
*/

template<class T >
class CkwsPlusLTRdmBuilder : public T
{ 
 public :
  /**
     \brief Destructor
   */
  ~CkwsPlusLTRdmBuilder();
  
  /**
     \brief Creates an instance of a Local Trees roadmap builder
     \param i_roadmap roadmap to work with
     \param i_penetration dynamic penetration
     \param i_evaluator distance evaluator
     \param i_picker diffusion node picker
     \param i_shooter diffusion shooter
     \return Shared pointer to a newly created roadmap builder
   */
  static 
    KIT_SHARED_PTR(CkwsPlusLTRdmBuilder<T>) 
    create(const CkwsRoadmapShPtr& i_roadmap, 
	   double i_penetration,
	   const CkwsMetricShPtr &i_evaluator = CkwsMetricEuclidean::create(),
	   const CkwsDiffusionNodePickerShPtr &i_picker = CkwsPickerBasic::create(), 
	   const CkwsDiffusionShooterShPtr &i_shooter = CkwsShooterConfigSpace::create());
  
  /**
     \brief Updates the connectivity of the roadmap by checking if the new node can be linked with a Connected Component. When a connection happens, it updates the list of diffusion nodes (trees' roots), removing when necessary (You can't have 2 diffusion nodes in the same connected component)
     \param i_node The newly created node (after an extension step).
     \param i_picked The picked diffusion node for this step (For removing it if a connection happens).
     \param isAShootedNode boolean saying if i_node is a shooted node (extension step reached the shooted node) or not.
     \return True if the update have succeed, false otherwise.
   */ 
  bool updateRoadmapConnectivity(CkwsNodeShPtr i_node, CkwsNodeShPtr i_picked, bool isAShootedNode);

  /**
     \brief Adding a diffusion node if the shooted configuration is not colliding, and if it cannot be connected with another component (this last is tested with updateConnectivity).
     \param i_newnode The newly created node.
     \param i_shootednode The shooted node.
     \param i_picked The picked diffusion node for this step.
     \return KD_OK
   */
  ktStatus addingDiffusionNodes(CkwsNodeShPtr i_newnode, CkwsNodeShPtr i_shootednode, CkwsNodeShPtr i_picked);

 protected :

  /**
     \brief Processing an extension step for a diffusion algorithm (RRT). It calls the base class' extend method
     \param i_node to extend from/to
     \param i_cfg to extend to/from
     \param i_direction of extension:
     * ROADMAP_TO_NODE: from i_node to i_cfg
     * NODE_TO_ROADMAP: from i_cfg to i_node
     * BIDIRECTIONAL: forbidden value, does nothing (throws an assert in debug mode)
     \return created node (can be null; failure is normal)

   */
  virtual CkwsNodeShPtr extend (const CkwsNodeShPtr& i_node, const CkwsConfig& i_cfg, CkwsRoadmapBuilder::EDirection i_direction);

  virtual CkwsNodeShPtr diffuse (const CkwsNodeShPtr &i_node, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type, CkwsRoadmapBuilder::EDirection &o_direction);

  /**
     \brief initialization
     \param i_weakPtr
     \return KD_OK | KD_ERROR 
   */
  ktStatus init(const KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) &i_weakPtr);

  /**
     \brief Constructor
     \param i_roadmap roadmap to work with
   */
  CkwsPlusLTRdmBuilder(const CkwsRoadmapShPtr &i_roadmap);

 private :

  KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) m_weakPtr;
  unsigned int oldStep;

};

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
bool CkwsPlusLTRdmBuilder<T>::updateRoadmapConnectivity(CkwsNodeShPtr i_node, CkwsNodeShPtr i_picked,bool isAShootedNode){

  CkwsNodeShPtr linkNode;
  CkwsDirectPathShPtr o_dp_in;
  CkwsDirectPathShPtr o_dp_out;
  bool success = false;
  
  for(unsigned int i=0;i<T::roadmap()->countConnectedComponents();i++){ //Checking if one or more of the existing connected components can be reached with i_node
    if(!success){ //cout<< "have not performed a connection yet." <<endl; getchar();
      if(i_node){ //cout<< "New Node exists!." <<endl;getchar();
	if(T::roadmap()->connectedComponent(i) != i_node->connectedComponent()){ //cout<< "Trying to reach another CC." <<endl;getchar();
	  if(T::canLinkNodeWithComponent ( CkwsRoadmapBuilder::NODE_TO_ROADMAP ,i_node, T::roadmap()->connectedComponent(i), CkitParameterMap::create(), linkNode, o_dp_in)
			|| T::canLinkNodeWithComponent ( CkwsRoadmapBuilder::ROADMAP_TO_NODE ,i_node, T::roadmap()->connectedComponent(i), CkitParameterMap::create(), linkNode, o_dp_out)){
	    //cout<< "Succeeding to reach another CC." <<endl;getchar();
	    bool continuer = false;
	  
	    CkwsDirectPathConstShPtr c_dp_in(CkwsDirectPath::createCopy(o_dp_in));

	    if(i_node && c_dp_in){
	      if(KD_OK != T::roadmapLink(i_node,linkNode,c_dp_in)) ;
	      else{ continuer=true; success = true; }
	    }

	    int sum=0;
	    for(unsigned int i=0;i<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE);i++){
	      
	      if(i_picked->connectedComponent() == T::diffusionNode(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE,i)->connectedComponent()){
		if(i_picked != T::diffusionNode(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE,i)) sum++;
	      }
	      
	    }
	    //cout<< "Found "<< sum<<"diffusin nodes in the picked CC." <<endl;getchar();
	    if(continuer || sum){//then update diffusion nodes list
	      //cout<<"Updating diffusion nodes list ("<< T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<" waypoint nodes). "<<endl;getchar();
	      if(!isAShootedNode){ //cout<<"is not a Shooted Node !"<<endl;getchar();
		std::vector<CkwsNodeShPtr> diffusion_nodes_list;
		for(unsigned int i=0; i<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE);i++){
		  diffusion_nodes_list.push_back(T::diffusionNode(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE,i));
		}
		T::resetDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE);
		//cout<<"temp list has "<<diffusion_nodes_list.size()<<" nodes"<<endl;getchar();
		//cout<<"initial list has "<<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<" nodes"<<endl;getchar();
		

		int found = false;
		
		for(unsigned int i=0; i<diffusion_nodes_list.size();i++){
		  if(!found){ //cout<<"not found"<<endl;getchar();//to avoid deletion of 2 diffusion nodes, and then having a connected component without a diffusion node.
		    if(!diffusion_nodes_list[i]->config().isEquivalent(i_picked->config())){ //cout<<" Not the same Config"<<endl;getchar();
		      if(diffusion_nodes_list[i]->connectedComponent() != i_picked->connectedComponent() ){ //cout<<" Not the same CC"<<endl;getchar();
			if(KD_OK != T::addDiffusionNode(diffusion_nodes_list[i],CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)){}//cout<<"Can't Add THE diffusion Node"<<endl;
			//else //cout<<"Added The diffusion Node"<<endl;
			//getchar();
		      }else{//cout<<"found a diffusion node in the same CC!"<<endl;
			found = true;
		      }//getchar();
		    }else{//cout<<"found the picked config!"<<endl;
		      found = true;
		    }//getchar();
		  }else if(KD_OK != T::addDiffusionNode(diffusion_nodes_list[i],CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)){}//cout<<"Can't Add one diffusion Node"<<endl;
			//else //cout<<"Add one diffusion Node"<<endl;
		  //getchar();
		}
		
		return true;
	      }//else //cout<<"is a Shooted Node !"<<endl;
	      //cout<< "After Update : "<<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<endl;getchar();
	    }//else cout<<"no need to update (no connection or every CC has only 1 D-Node)"<<endl;getchar();
	  }//else cout<<"Can't link node with any CC (means that the can potientially be a diffusio node)"<<endl;getchar();
	}//else cout<<"We are in the same CC as the new node"<<endl;getchar();
      }//else cout<< "New Node does not exists!." <<endl;getchar();
    }//else cout<< "have performed a connection." <<endl;getchar();
  }

  return success;
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


template <class T>
CkwsPlusLTRdmBuilder<T>::~CkwsPlusLTRdmBuilder(){
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
ktStatus CkwsPlusLTRdmBuilder<T>::addingDiffusionNodes(CkwsNodeShPtr i_newnode, CkwsNodeShPtr i_shootednode, CkwsNodeShPtr i_picked){

  if(i_newnode){ //cout<<"New Node Created! "<<endl;getchar();
    if(i_newnode != i_shootednode){ //cout<<"- Shooted Node Not Reached! "<<endl;getchar();
      bool linked = false;
      bool success = false;
      if(T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<10){ //cout<<"- Can Add Diffusion Node! "<<endl;getchar();
	if(KD_OK == T::addNode(i_shootednode)){ //cout<<"- Adding Node! "<<endl;getchar();
	  success = true; 
	  linked = updateRoadmapConnectivity(i_shootednode,i_picked,true);
	  if(!linked){ //cout<<"- Adding Diffusion Node! "<<endl;getchar();
	    if(KD_OK == T::addDiffusionNode(i_shootednode,CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)){ //cout<<"- OK : "<< T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<"! "<<endl;
	      if(KD_OK != T::roadmap()->device()->setCurrentConfig(i_shootednode->config())){} //cout<<"unable to set current config"<<endl;getchar();
	     }else{ //cout<<"- Can't Add a Diffusion Node"<<endl;getchar();
	      if(KD_OK != T::roadmap()->removeNode(i_shootednode)){} //cout<<"haven't removed the shooted node"<<endl;
	    	//else cout<<"have removed the shooted node"<<endl;
	    	//getchar();
	    }
	  }//else cout<<"- No Diffusion Node Created! "<<endl;getchar();
	}//else cout<<"- No Node Added! "<<endl;getchar();
      }//else cout<<"- Already 10 Diffusion Node! "<<endl;getchar();
    }//else cout<<"- Shooted Node Reached! "<<endl;getchar();
  }//else cout<<"- No New Node! "<<endl;getchar();
  
    return KD_OK;
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
CkwsNodeShPtr CkwsPlusLTRdmBuilder<T>::extend (const CkwsNodeShPtr& i_node, const CkwsConfig& i_cfg, CkwsRoadmapBuilder::EDirection i_direction){

//cout<<"Extend Begin -----"<<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<"-------------------------------------------------------"<<endl;
  bool collision = true;

  if(T::roadmap()->countNodes() < oldStep){
    oldStep=T::roadmap()->countNodes();
    T::injectWaypointNodes();
  }
	bool outExtend = false;
  CkwsNodeShPtr newNode = T::extend(i_node,i_cfg,i_direction, CkitParameterMap::create(),outExtend);
  if(!newNode){;
  }else{ 
    updateRoadmapConnectivity(newNode,i_node,false);
  }
  if(KD_OK == i_cfg.isColliding(collision)){
    if(!collision){
      addingDiffusionNodes(newNode,CkwsNode::create(i_cfg),i_node);
    }
  }
  oldStep=T::roadmap()->countNodes();
//cout<<"Extend End -------"<<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<"-------------------------------------------------------"<<endl;
//system(clear);
	//getchar();
  return newNode;

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
template <class T>
CkwsNodeShPtr CkwsPlusLTRdmBuilder<T>::diffuse (const CkwsNodeShPtr &i_node, CkwsDiffusingRdmBuilder::EDiffusionNodeType i_type, CkwsRoadmapBuilder::EDirection &o_direction){

//   if(i_type == CkwsDiffusingRdmBuilder::START_LIKE) cout<<"Type d'arbre : START"<<endl;
//   else if(i_type == CkwsDiffusingRdmBuilder::GOAL_LIKE) cout<<"Type d'arbre : GOAL"<<endl;
//   else if(i_type == CkwsDiffusingRdmBuilder::WAYPOINT_LIKE) cout<<"Type d'arbre : WAYPOINT"<<endl;
// 
//   cout<<"Nb diffusion Nodes (START):    "<< T::countDiffusionNodes(CkwsDiffusingRdmBuilder::START_LIKE)<<endl;
//   cout<<"Nb diffusion Nodes (GOAL):     "<< T::countDiffusionNodes(CkwsDiffusingRdmBuilder::GOAL_LIKE)<<endl;
//   cout<<"Nb diffusion Nodes (WAYPOINT): "<< T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<<endl;

  CkwsNodeShPtr result = T::diffuse(i_node,i_type,o_direction);
  return result;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
CkwsPlusLTRdmBuilder<T>::CkwsPlusLTRdmBuilder(const CkwsRoadmapShPtr &i_roadmap): T(i_roadmap){
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
ktStatus CkwsPlusLTRdmBuilder<T>::init(const KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) &i_weakPtr){

  ktStatus res = T::init(i_weakPtr);

  if(res == KD_OK){
    oldStep = 0;
    m_weakPtr = i_weakPtr;
  }

  return res;

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
KIT_SHARED_PTR(CkwsPlusLTRdmBuilder<T>) CkwsPlusLTRdmBuilder<T>::create(const CkwsRoadmapShPtr &i_roadmap, double i_penetration,const CkwsMetricShPtr &i_evaluator, const CkwsDiffusionNodePickerShPtr &i_picker, const CkwsDiffusionShooterShPtr &i_shooter)
{

  //cout<<endl;

  CkwsPlusLTRdmBuilder<T>* rdmBuilderPtr = new CkwsPlusLTRdmBuilder(i_roadmap);
  KIT_SHARED_PTR(CkwsPlusLTRdmBuilder<T>) rdmBuilderShPtr(rdmBuilderPtr);
  KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) rdmBuilderWkPtr(rdmBuilderShPtr);

  if(KD_OK != rdmBuilderPtr->init(rdmBuilderWkPtr)){
    rdmBuilderShPtr.reset();
  }
  else {
    rdmBuilderShPtr->roadmap ()->configSpace ()->metric (i_evaluator);
    rdmBuilderShPtr->diffusionNodePicker(i_picker);
    rdmBuilderShPtr->diffusionShooter(i_shooter);
    if(KD_ERROR == CkwsValidatorDPCollision::setPenetration(rdmBuilderShPtr->builderDirectPathValidator(), i_penetration)) {
      rdmBuilderShPtr.reset();
    }
  }

  return rdmBuilderShPtr;

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

typedef  CkwsPlusLTRdmBuilder<CkwsDiffusingRdmBuilder> CkwsPlusLTDiffusingRdmBuilder;
typedef  CkwsPlusLTRdmBuilder<CkwsIPPRdmBuilder> CkwsPlusLTIPPRdmBuilder;

KIT_POINTER_DEFS(  CkwsPlusLTDiffusingRdmBuilder );
KIT_POINTER_DEFS(  CkwsPlusLTIPPRdmBuilder );

/**
   @}
*/

#endif //KWS_PLUS_LT_ROADMAP_BUILDER
