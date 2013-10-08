/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Sebastien Dalibard (LAAS-CNRS)
*/


#ifndef KWS_PLUS_KPCA_ROADMAP_BUILDER
#define KWS_PLUS_KPCA_ROADMAP_BUILDER

/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsValidatorDPCollision.h"
#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDiffusionNodePicker.h"
#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoWorks2/kwsPickerBasic.h"
#include "KineoWorks2/kwsShooterRoadmapBox.h"
#include "KineoWorks2/kwsConnectedComponent.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsDof.h"
#include "KineoWorks2/kwsEdge.h"

#include "KineoUtility/kitInterface.h"
#include "KineoWorks2/kwsDefine.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <stdio.h>                                                                                    
#include <sys/time.h>  

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/bindings/lapack/syev.hpp>
#include "boost/numeric/bindings/traits/ublas_matrix.hpp"
#include "boost/numeric/bindings/traits/ublas_vector.hpp"

#define SIZE_DATA 3

namespace ublas = boost::numeric::ublas ;
namespace lapack = boost::numeric::bindings::lapack ;
  
typedef struct CkwsDistNodeShPtr {
    CkwsNodeShPtr Ptr;
    double Dist;
    static bool DistNodePred(CkwsDistNodeShPtr X, CkwsDistNodeShPtr Y) { return X.Dist>Y.Dist; }       
} CkwsDistNodeShPtr;

typedef struct IthEigenValue {
    double eigenvalue;
    unsigned int i;
    static bool IthEigenValuePred(IthEigenValue X, IthEigenValue Y) { return X.eigenvalue>Y.eigenvalue; }       
} IthEigenValue;

typedef queue<CkwsNodeShPtr> nodesQueue;

KIT_PREDEF_CLASS( CkwsRoadmap );


template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusKPCARdmBuilder; 

class CkwsDiffusingRdmBuilder;
class CkwsIPPRdmBuilder;


#if 0
typedef  CkwsPlusKPCARdmBuilder<CkwsDiffusingRdmBuilder> CkwsPlusPCADiffusingRdmBuilderPerso;
typedef  CkwsPlusKPCARdmBuilder<CkwsIPPRdmBuilder> CkwsPlusPCAIPPRdmBuilderPerso;

KIT_POINTER_DEFS(  CkwsPlusPCADiffusingRdmBuilderPerso );
KIT_POINTER_DEFS(  CkwsPlusPCAIPPRdmBuilderPerso );
#endif

/**
   \addtogroup PCA
   @{
This part implements a Kernel Principal Component Analysis (KPCA) based roadmap building strategy.

\section kpca Kernel Principal Components Analysis (KPCA)
The KPCA is a statistical technique used to reduce multidimensional 
data sets to lower dimension. It generalizes the PCA for non linear 
dimensionnality reduction by using the kernel trick. KPCA can be used 
for the detection of submanifold generated by any steering method.
The way KPCA uses the steering method is by calling the roadmap builder
function distance(), which should therefore suit the steering method.


/**
 \brief This template class inherits from any diffusing roadmap builder class,
 and transforms the diffusion process by taking into account the results 
 of a principal components analysis of the nodes made at each step of
 diffusion
*/


template<class T >

class CkwsPlusKPCARdmBuilder : public T
{
 public:

/**
     \brief Destructor.
 */
  ~CkwsPlusKPCARdmBuilder();

 /**
     \brief Constructor.
     \param i_roadmap The roadmap to construct.
     \param i_penetration The penetration allowed.
     \param i_evaluator The class used to evaluate the distance between two configurations.
     \param i_picker The picker used to pick diffusion nodes.
     \param i_shooter The shooter used to generate the configurations.
 */

  static 
    KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>)
    create(const CkwsRoadmapShPtr& i_roadmap,
	   double i_penetration,
//	   const CkwsDistanceShPtr& i_evaluator = CkwsDistance::create(),
//	   const CkwsDiffusionNodePickerShPtr &i_picker=CkwsDiffusionNodePickerShPtr(), 
//	   const CkwsDiffusionShooterShPtr &i_shooter=CkwsDiffusionShooterShPtr());

	   const CkwsDistanceShPtr& i_evaluator = CkwsDistance::create(),
	   const CkwsDiffusionNodePickerShPtr &i_picker = CkwsPickerBasic::create(), 
	   const CkwsDiffusionShooterShPtr &i_shooter = CkwsShooterRoadmapBox::create());


/**
     \brief Copy constructor.
     \param inRdmpBuilder The roadmap builder to copy.
*/
  static 
    KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>)
    createCopy(const  KIT_SHARED_PTR_CONST(CkwsPlusKPCARdmBuilder<T>) &inRdmpBuilder);



 protected:
  /**
     \brief Constructor
     \param i_roadmap: The roadmap to be built,
  */
  CkwsPlusKPCARdmBuilder(const CkwsRoadmapShPtr& i_roadmap);


  /**
     \brief Init function
     \param i_weakPtr The weak pointer to the PCARdmBuilder.
     \return ktStatus KD_OK or KD_ERROR
  */

  ktStatus init(const KIT_WEAK_PTR(CkwsPlusKPCARdmBuilder<T>)& i_weakPtr);


  /**
     \brief Extend function: extends the roadmap to/from a node to/from 
     a random configuration. Here, we first call doPCA before calling the
     standard diffusing extend function with the new random configuration.
     \param i_node Node to extend from/to.
     \param i_cfg Configuration to extend to/from. 
     \param i_direction ROADMAP_TO_NODE or NODE_TO_ROADMAP
     \return The new node of the roadmap (can be NULL)
  */
  CkwsNodeShPtr extend(const CkwsNodeShPtr & i_node,const CkwsConfig & i_cfg,CkwsRoadmapBuilder::EDirection i_direction );

  /**
     \brief doPCA function: first does a Breadth-First Search in the roadmap 
     starting from the node to extend from, when it has got enough nodes
     performs a Kernel Principal Components Analysis on these nodes and modifies the
     random configuration according to that analysis.
     \param fatherNode Node to extend from.
     \param qRand Initial random configuration.
     \param minNb min number of nodes needed to do a PCA
     \param resultCfg pointer to a Config where the new 
     direction to extend to will be stored
  */
  void doPCA(const CkwsNodeShPtr fatherNode, 
	     const CkwsConfig qRand,
	     unsigned int minNb,
	     CkwsConfig * resultCfg,
	     nodesQueue * nodesSet);
	     
  double kernel(ublas::vector<double> VectX,
                ublas::vector<double> VectY);

 private:
  KIT_WEAK_PTR(CkwsPlusKPCARdmBuilder<T>) m_weakPtr;
  CkwsRoadmapShPtr m_roadmap;

};



template<class T>
CkwsPlusKPCARdmBuilder<T>::~CkwsPlusKPCARdmBuilder()
{
}



template<class T>
KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>)
  CkwsPlusKPCARdmBuilder<T>::create(
				   const CkwsRoadmapShPtr &i_roadmap,
				   double i_penetration,
				   const CkwsMetricShPtr &i_evaluator,
				   const CkwsDiffusionNodePickerShPtr &i_picker, 
				   const CkwsDiffusionShooterShPtr &i_shooter
				   )
{
  cout << "CkwsPlusKPCARdmBuilder::create" << endl;
  CkwsPlusKPCARdmBuilder<T> * pcaRdmBuilderPtr = new CkwsPlusKPCARdmBuilder<T>(i_roadmap);
  KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>) pcaRdmBuilderShPtr(pcaRdmBuilderPtr);
  KIT_WEAK_PTR(CkwsPlusKPCARdmBuilder<T>) pcaRdmBuilderWkPtr(pcaRdmBuilderShPtr);

  if (pcaRdmBuilderPtr->init(pcaRdmBuilderWkPtr) != KD_OK){
    pcaRdmBuilderShPtr.reset();
  }
  else {
    pcaRdmBuilderPtr->roadmap ()->configSpace ()->metric (i_evaluator);
//    pcaRdmBuilderPtr->diffusionNodePicker(i_picker);
//    pcaRdmBuilderPtr->diffusionShooter(i_shooter);
    if(KD_ERROR == CkwsValidatorDPCollision::setPenetration(pcaRdmBuilderShPtr->builderDirectPathValidator(), i_penetration)) {
      pcaRdmBuilderShPtr.reset();
    }
  }

  return pcaRdmBuilderShPtr;

}

template<class T>
KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>)
  CkwsPlusKPCARdmBuilder<T>::createCopy(const KIT_SHARED_PTR_CONST(CkwsPlusKPCARdmBuilder<T>) &inRdmpBuilder)
{
  if(inRdmpBuilder != NULL) {
    CkwsPlusKPCARdmBuilder<T>* pcaRdmBuilderPtr = new CkwsPlusKPCARdmBuilder<T>(*inRdmpBuilder);
    KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>) pcaRdmBuilderShPtr(pcaRdmBuilderPtr);
    KIT_WEAK_PTR(CkwsPlusKPCARdmBuilder<T>) pcaRdmBuilderWkPtr(pcaRdmBuilderShPtr);

    if(pcaRdmBuilderPtr->init(pcaRdmBuilderWkPtr) != KD_OK){
      pcaRdmBuilderShPtr.reset();
    }
    else {
      pcaRdmBuilderPtr->distance(inRdmpBuilder->distance());
      double penetration;
      if (KD_ERROR == CkwsValidatorDPCollision::getPenetration(inRdmpBuilder->builderDirectPathValidator(), penetration)) {
	pcaRdmBuilderShPtr.reset();
      }
      else {
	if(KD_ERROR == CkwsValidatorDPCollision::setPenetration(pcaRdmBuilderShPtr->builderDirectPathValidator(), penetration)) {
	  pcaRdmBuilderShPtr.reset();
	}
      }
    }
    return pcaRdmBuilderShPtr;
  }
  return KIT_SHARED_PTR(CkwsPlusKPCARdmBuilder<T>)();
}


template<class T>
CkwsPlusKPCARdmBuilder<T>::CkwsPlusKPCARdmBuilder(const CkwsRoadmapShPtr& i_roadmap)
  : T(i_roadmap)
{

}


template<class T>
ktStatus CkwsPlusKPCARdmBuilder<T>::init(const KIT_WEAK_PTR(CkwsPlusKPCARdmBuilder<T>)& i_weakPtr)
{
  ktStatus success = T::init(i_weakPtr);
  if (KD_OK == success){
    m_weakPtr = i_weakPtr;
  } 
  return success ;
}


template<class T>
CkwsNodeShPtr CkwsPlusKPCARdmBuilder<T>::extend(const CkwsNodeShPtr & i_node,
					       const CkwsConfig & i_cfg,
					       CkwsRoadmapBuilder::EDirection i_direction )
{
  unsigned int n = 0;
  for(unsigned int i=0;i<T::roadmap()->device()->countDofs();i++)
    if(!T::roadmap()->device()->dof(i)->isLocked())
      n++;
  

  double do_project = (double) rand()/(INT_MAX);
  double pb = 0.5;
  
  bool projected = false;
  CkwsConfig newCfg(T::roadmap()->device());
  nodesQueue nodesSet;
 
  if(do_project<pb)
    {
      if(i_node->connectedComponent()->countNodes()> SIZE_DATA*n)
	    {
	      doPCA(i_node,i_cfg,SIZE_DATA*n,&newCfg,&nodesSet);
	      projected = true;
	    }
    }

  CkwsNodeShPtr newNode;
  
  if(projected)
    {
      newNode = T::extend(i_node,newCfg,i_direction);
      
      /*if(newNode != NULL)
      {
        CkwsDistNodeShPtr Temp;
        queue<CkwsDistNodeShPtr> nodes;
        CkwsNodeShPtr nearestNode;
        double minDist=-1.0;
            
        while(!nodesSet.empty())
        {
          Temp.Ptr = nodesSet.front();
          Temp.Dist = T::roadmap()->device()->distance()->distance(newNode->config(), Temp.Ptr->config());
          nodes.push(Temp);
          nodesSet.pop();
        }
        while(!nodes.empty())
        {
          if(minDist==-1.0 || nodes.front().Dist<minDist)
          {
            nearestNode=nodes.front().Ptr;
            minDist=nodes.front().Dist;
          }  
          nodes.pop();
        }
        if(nearestNode!=NULL)
        {
          CkwsDirectPathShPtr Path = T::roadmap()->device()->steeringMethod()->makeDirectPath(nearestNode->config(),newNode->config());
          if(Path!=NULL)
          {
            bool result = T::roadmap()->device()->directPathValidators()->validate(*Path);  
            if(result)
            {
              T::addEdge(nearestNode,newNode,(CkwsDirectPathConstShPtr)Path); 
              T::addEdge(newNode,nearestNode,(CkwsDirectPathConstShPtr)CkwsDirectPath::createReversed(Path));                  
            }  
          }
        }
      }*/
    }
  else 
    newNode = T::extend(i_node,i_cfg,i_direction);      
    
  return newNode;  
}

template<class T>
double CkwsPlusKPCARdmBuilder<T>::kernel(ublas::vector<double> VectX, ublas::vector<double> VectY)
{
  CkwsConfig O(T::roadmap()->device());
  CkwsConfig X(T::roadmap()->device());
  CkwsConfig Y(T::roadmap()->device());  
  
  for(unsigned int i=0;i<T::roadmap()->device()->countDofs();i++)
  {
    O.dofValue(i,0);
    X.dofValue(i,VectX(i));
    Y.dofValue(i,VectY(i));    
  }  
  
  double OX = T::roadmap()->device()->distance()->distance(O,X);
  double OY = T::roadmap()->device()->distance()->distance(O,Y);
  double XY = T::roadmap()->device()->distance()->distance(X,Y);
  
  return (OX*OX+OY*OY-XY*XY)/2;
}

template<class T>
void CkwsPlusKPCARdmBuilder<T>::doPCA(const CkwsNodeShPtr fatherNode, 
				     const CkwsConfig qRand,
				     unsigned int minNb,
				     CkwsConfig * resultCfg,			     
      	     nodesQueue * nodesSet)
{

  unsigned int i,j,k;    
  
  unsigned int n = 0;
  for(unsigned int i=0;i<T::roadmap()->device()->countDofs();i++)
    if(!T::roadmap()->device()->dof(i)->isLocked())
      n++;
      
  vector< ublas::vector<double> > tab;      
  ublas::vector<double> mean_tab(T::roadmap()->device()->countDofs());  
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    mean_tab(k)=0.0;

  /* Breadth-first search to find the neighbours used in the PCA*/
  CkwsDistNodeShPtr Temp;
  
  map<CkwsNodeShPtr , int> color;
  map<CkwsNodeShPtr , int> distance;
  
  /*typedef vector<CkwsDistNodeShPtr> NodeVector ;
  typedef NodeVector::iterator NodeVectorIt ;
  NodeVectorIt it ;*/
    
  vector<CkwsDistNodeShPtr> F;
  
  unsigned int neighbours = 0;
  int d_max = INT_MAX;

  Temp.Ptr=fatherNode;
  Temp.Dist=0.;
  F.push_back(Temp);
  color[fatherNode] = 1;
  distance[fatherNode] = 0;

  while(!F.empty()) {      
    
    make_heap(F.begin(), F.end(), CkwsDistNodeShPtr::DistNodePred);
    pop_heap(F.begin(), F.end(), CkwsDistNodeShPtr::DistNodePred);    
    CkwsNodeShPtr current = F.back().Ptr;    
    F.pop_back();
  
    if(neighbours)  /* We don't want the fathernode */
      nodesSet->push(current);
        
    if (distance[current] > d_max)
      break; 
    
    for(i = 0; i< current->countOutEdges();i++){
      CkwsNodeShPtr son = current->outEdge(i)->endNode();
      if ((color[son] != 1)&&(color[son] != 2)){
	      color[son] = 1;
	      distance[son] = distance[current] +1;
	      Temp.Ptr = son;
	      Temp.Dist = T::roadmap()->device()->distance()->distance(fatherNode->config(),son->config());
	      F.push_back(Temp);
	      push_heap(F.begin(), F.end(), CkwsDistNodeShPtr::DistNodePred); 
	      make_heap(F.begin(), F.end(), CkwsDistNodeShPtr::DistNodePred);   
      }
    }

    neighbours++;
    if(neighbours > minNb)
      d_max = distance[current];      
      
    ublas::vector<double> Xi(T::roadmap()->device()->countDofs());
    for(k=0;k<T::roadmap()->device()->countDofs();k++)
      Xi(k)=current->config().dofValue(k);          
    tab.push_back(Xi);
    mean_tab += (Xi - mean_tab) / neighbours;

    color[current] = 2;
  }
  
  /* for security, should not occur */
  if(neighbours<n) n=neighbours;
  
//{ //Pour comparer avec la PCA  
  ublas::matrix<double, ublas::column_major> K(neighbours,neighbours); //kernel matrix 
  vector<double> meansK(neighbours);
  double totMeanK = 0.0;
  
  /* We center the data */
  for(i=0;i<neighbours;i++)
    tab[i] -= mean_tab;
  
  /* We compute te kernel matrix */
  for(i=0;i<neighbours;i++)
  {
    K(i,i) = kernel(tab[i],tab[i]);
    meansK[i] = 0.0;
  }
  for(i=0;i<neighbours;i++)
    for(j=0;j<i;j++)
    {
      K(i,j) = kernel(tab[i],tab[j]);
      K(j,i) = K(i,j);      
    }
    
  /* Then we centrate the matrix */
  for(i=0;i<neighbours;i++)
    for(j=0;j<neighbours;j++)
    {
      meansK[i] += (K(i,j) - meansK[i]) / (j+1);
      totMeanK += (K(i,j) - totMeanK) / (i*neighbours+j+1);
    }   
  for(i=0;i<neighbours;i++)
    K(i,i) += totMeanK - 2*meansK[i];
  for(i=0;i<neighbours;i++)
    for(j=0;j<i;j++)
    {
      K(i,j) += totMeanK - meansK[i] - meansK[j];
      K(j,i) = K(i,j);      
    }
      
  /* We need to find the eigenvalues of K/neighbours  */
  K /= ((double)neighbours);

  ublas::vector<double> eigenValues(neighbours);
  vector<IthEigenValue> sortedEV;
    
  lapack::syev('V',
	       'L',
	       K,
	       eigenValues,
	       lapack::optimal_workspace());

  /*printf("eigenvalues:  (");
  for(k=0;k<neighbours;k++)
    cout << eigenValues(k)  << ", ";
  cout << ")"  << endl;*/
  
  double max = 0;
  for(i=0;i<neighbours;i++){
    if(eigenValues[i] >max)
      max = eigenValues[i];
    IthEigenValue EV;
    EV.eigenvalue = eigenValues[i];
    EV.i=i;
    sortedEV.push_back(EV);
  }
  
  sort(sortedEV.begin(), sortedEV.end(), IthEigenValue::IthEigenValuePred);
  
  CkwsConfig qNear = fatherNode->config();
  ublas::vector<double> vectNearToRand(T::roadmap()->device()->countDofs());
  ublas::vector<double> newVectNearToRand(T::roadmap()->device()->countDofs());
  
  for(i = 0; i<T::roadmap()->device()->countDofs(); i++) {
    vectNearToRand(i) = qRand.dofValue(i) - qNear.dofValue(i); 
    newVectNearToRand(i) = 0.0;
  }   
        
  double mean = 0;
  for(i = 0; i<neighbours; i++)
    mean += (kernel(vectNearToRand, tab[i]) - mean) / (i+1); 
   
  for(i = 0; i<neighbours; i++) { 
    ublas::vector<double> eigenVector(neighbours);    
    for(j=0;j<neighbours;j++) {
      eigenVector(j) = K(j,sortedEV[i].i);
    }
    
    double projection = 0;
    for(j=0;j<neighbours;j++) {
      projection += eigenVector[j] * (kernel(vectNearToRand, tab[j]) - mean);
    }
        
    /*
    double coeff_norme = (eigenvalue/max) / ||V_k||�));  // == (eigenvalue/max) / (neighbours *eigenvalue * ||eigenvector||�) ==  1 / (max * neighbours * ||eigenvector||�)
    double coeff = coeff_norme * projection;*/
    
    // The norm of the eigenvectors is 1
    double coeff = projection / (((double)neighbours) * max);
    
    eigenVector *= coeff;
    
    for(j=0;j<neighbours;j++) 
      newVectNearToRand += eigenVector(j) * tab[j];
  }
  
  double norme = inner_prod(newVectNearToRand,newVectNearToRand);
  
  if(norme!=0)
    newVectNearToRand *= inner_prod(vectNearToRand,vectNearToRand) / norme;
  
  for(k = 0; k<T::roadmap()->device()->countDofs(); k++) 
    resultCfg->dofValue(k, qNear.dofValue(k) + newVectNearToRand(k));
     
  /*printf("VectNearToRand:  (");
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    cout << vectNearToRand(k) << ", ";
  cout << ")"  << endl;
     
  printf("newVectNearToRand:  (");
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    cout << newVectNearToRand(k) << ", ";
  cout << ")"  << endl;*/
        
 /* printf("Old pos:  (");
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    cout << qRand.dofValue(k) << ", ";
  cout << ")"  << endl;
  
  printf("Old vecteur:  (");
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    cout << vectNearToRand.dofValue(k) << ", ";
  cout << ")"  << endl;
    
  printf("Nv vecteur:  (");
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    cout << (resultCfg->dofValue(k) - qNear.dofValue(k))  << ", ";
  cout << ")"  << endl ;
  
  printf("Nv pos:  (");
  for(k=0;k<T::roadmap()->device()->countDofs();k++)
    cout << resultCfg->dofValue(k)  << ", ";
  cout << ")"  << endl ;*/

 /* printf("Total:  (");
  nbr++;
  if(total.size()!=T::roadmap()->device()->countDofs())
  {
    total.resize(T::roadmap()->device()->countDofs());
    for(i=0;i<T::roadmap()->device()->countDofs();i++)
      total[i]=0.;
  }
  double factor=1;
  if((resultCfg->dofValue(0) - qNear.dofValue(0))<0)
    factor=-1;
  for(i=0;i<T::roadmap()->device()->countDofs();i++)
  {
    total[i] += (factor*(resultCfg->dofValue(i) - qNear.dofValue(i))- total[i]) / nbr;
    cout << (total[i])  << ", ";
  }  
  cout << ")"  << endl << endl;*/
/*}
{
  int n = T::roadmap()->device()->countDofs();
  vector<double> * tab2 = new vector<double>[n];  
  ublas::matrix<double, ublas::column_major> cov (n,n); //covariance matrix 
  double * means = new double[n];
    
  for(i=0;i<n;i++)
    for(k=0;k<neighbours;k++)  
      tab2[i].push_back(tab[k](i));

  // First we compute the estimated mean vector. 
  for(i=0;i<n;i++){
    double meanI = 0;
    for(k=0;k<neighbours;k++){
      meanI += (tab2[i][k] - meanI)/ (k +1);
    }
    means[i] = meanI;
  }

  // We then compute the estimated variances... 
  for(i=0;i<n;i++){
    double variance = 0;
    for(k=0;k<neighbours;k++){
      double delta = tab2[i][k] - means[i];
      variance += (delta * delta - variance) / (k+1);
    }
    cov (i,i) = variance *((double) neighbours / (double)(neighbours-1));
  }

  // ...and the estimated covariances. 
  for(i=0;i<n;i++){
    for(j=0;j<i;j++){
      double covariance = 0;
      for(k=0;k<neighbours;k++){
	double delta1 = tab2[i][k] - means[i];
	double delta2 = tab2[j][k] - means[j];
	covariance += (delta1 * delta2 - covariance) / (k+1);
      }
      covariance *= (double) neighbours / (double) (neighbours -1);
      cov (i,j) = covariance;
      cov (j,i) = covariance;
    }
  }
  
  ublas::vector<double> eigenValues(n);
    
  lapack::syev('V',
	       'L',
	       cov,
	       eigenValues,
	       lapack::optimal_workspace());

  double max = 0;
  for(i=0;i<n;i++){
    if(eigenValues[i] >max)
      max = eigenValues[i];
  }

 CkwsConfig qNear = fatherNode->config();
 ublas::vector<double> randomVector (n);
 ublas::vector<double> newRandomVector(n);
  for(i = 0; i<n ; i++){
    double randI = qRand.dofValue(i) - qNear.dofValue(i);
    randomVector(i) = randI;
    newRandomVector(i) = 0.;
  }

  for(i = 0;i<n;i++){
    double coeff = eigenValues[i] / max;
    ublas::vector<double> eigenVector (n);
    
    for(j=0;j<n;j++){
      eigenVector(j) = cov(j,i);
    }
    
    double alpha = inner_prod(randomVector, eigenVector);
    newRandomVector += alpha*coeff*eigenVector;
  }
  
  for(i=0;i<n;i++){
    double dofI = qNear.dofValue(i) + newRandomVector(i);
    resultCfg->dofValue(i,dofI);
  }
  
    
  printf("newVectNearToRand PCA:  (");
  for(k=0;k<n;k++)
    cout << newRandomVector(k) << ", ";
  cout << ")"  << endl;
    
  printf("Total:  (");
  nbr++;
  if(total.size()!=T::roadmap()->device()->countDofs())
  {
    total.resize(T::roadmap()->device()->countDofs());
    for(i=0;i<T::roadmap()->device()->countDofs();i++)
      total[i]=0.;
  }
  double factor=1;
  if((resultCfg->dofValue(0) - qNear.dofValue(0))<0)
    factor=-1;
  for(i=0;i<T::roadmap()->device()->countDofs();i++)
  {
    total[i] += (factor*(resultCfg->dofValue(i) - qNear.dofValue(i))- total[i]) / nbr;
    cout << (total[i])  << ", ";
  }  
  cout << ")"  << endl << endl;
  
  
  delete [] tab2;
  delete [] means;
}*/
}

typedef  CkwsPlusKPCARdmBuilder<CkwsDiffusingRdmBuilder > CkwsPlusPCADiffusingRdmBuilderPerso;
typedef  CkwsPlusKPCARdmBuilder<CkwsIPPRdmBuilder> CkwsPlusPCAIPPRdmBuilderPerso;


KIT_POINTER_DEFS(  CkwsPlusPCADiffusingRdmBuilderPerso );
KIT_POINTER_DEFS(  CkwsPlusPCAIPPRdmBuilderPerso );

/**
   @}
*/

#endif