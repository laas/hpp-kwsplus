/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Sebastien Dalibard (LAAS-CNRS)
*/


#ifndef KWS_PLUS_PCA_ROADMAP_BUILDER
#define KWS_PLUS_PCA_ROADMAP_BUILDER

/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsDistance.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDiffusionNodePicker.h"
#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoWorks2/kwsConnectedComponent.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsDof.h"
#include "KineoWorks2/kwsEdge.h"

#include "KineoUtility/kitInterface.h"
#include "KineoWorks2/kwsDefine.h"

#include <vector>
#include <cmath>
#include <queue>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/bindings/lapack/syev.hpp>
#include "boost/numeric/bindings/traits/ublas_matrix.hpp"
#include "boost/numeric/bindings/traits/ublas_vector.hpp"


using namespace std;

KIT_PREDEF_CLASS( CkwsRoadmap );


template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusPCARdmBuilder; 

class CkwsDiffusingRdmBuilder;
class CkwsIPPRdmBuilder;


#if 0
typedef  CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder> CkwsPlusPCADiffusingRdmBuilder;
typedef  CkwsPlusPCARdmBuilder<CkwsIPPRdmBuilder> CkwsPlusPCAIPPRdmBuilder;

KIT_POINTER_DEFS(  CkwsPlusPCADiffusingRdmBuilder );
KIT_POINTER_DEFS(  CkwsPlusPCAIPPRdmBuilder );
#endif

/**
   \addtogroup PCA
   @{
This part implements a Principal Component Analysis (PCA) based roadmap building strategy.

\section pca Principal Components Analysis (PCA)
The PCA is a statistical technique used to reduce multidimensional 
data sets to lower dimension.

By computing an eigenvectors decomposition of the covariance matrix of the data 
set, the PCA retains the dimension of the space that contribute most to the variance
of the set.

\image html pca.png "Exemple of PCA on a 2D data set" 

\section usePCA Use of the PCA in diffusing algorithms
A random tree diffusing through a narrow passage can be slowed down by collisions 
occuring along a given direction, while other directions are free. At each step 
of diffusion, we performe a PCA on the nearest neighbours of the node to extend 
the roadmap from. We then change the random configuration to which the tree is 
extended to favor directions corresponding to high variances. 

\image html pcaDiff.png "Transforming the random configuration after having performed a PCA"

\anchor rapport-sebastien
S. Dalibard, <a href="./papers/rapport-sebastien.pdf">"Métriques de diffusion en planification probabiliste de mouvement"</a> Rapport M2 MPRI, Septembre 2007
*/

/**
 \brief This template class inherits from any diffusing roadmap builder class,
 and transforms the diffusion process by taking into account the results 
 of a principal components analysis of the nodes made at each step of
 diffusion
*/


template<class T >
class CkwsPlusPCARdmBuilder : public T
{
 public:

/**
     \brief Destructor.
 */
  ~CkwsPlusPCARdmBuilder();

 /**
     \brief Constructor.
     \param i_roadmap The roadmap to construct.
     \param i_penetration The penetration allowed.
     \param i_evaluator The class used to evaluate the distance between two configurations.
     \param i_picker The picker used to pick diffusion nodes.
     \param i_shooter The shooter used to generate the configurations.
 */

  static 
    KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
    create(const CkwsRoadmapShPtr& i_roadmap,
	   double i_penetration,
	   const CkwsDistanceShPtr& i_evaluator = CkwsDistance::create(),
	   const CkwsDiffusionNodePickerShPtr &i_picker=CkwsDiffusionNodePickerShPtr(), 
	   const CkwsDiffusionShooterShPtr &i_shooter=CkwsDiffusionShooterShPtr());
/**
     \brief Copy constructor.
     \param inRdmpBuilder The roadmap builder to copy.
     \param i_penetration The penetration allowed.
*/
  static 
    KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
    createCopy(const  KIT_SHARED_PTR_CONST(CkwsPlusPCARdmBuilder<T>) &inRdmpBuilder, 
	       double i_penetration);



 protected:
  /**
     \brief Constructor
     \param i_roadmap: The roadmap to be built,
     \param i_evaluator: The class used to evaluate the distance between two configurations.
  */
  CkwsPlusPCARdmBuilder(const CkwsRoadmapShPtr& i_roadmap, 
			const CkwsDistanceShPtr& i_evaluator = CkwsDistance::create());


  /**
     \brief Init function
     \param i_weakPtr The weak pointer to the PCARdmBuilder.
     \param i_penetration The penetration allowed.
     \return ktStatus KD_OK or KD_ERROR
  */

  ktStatus init(const KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>)& i_weakPtr, double i_penetration);


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
     performs a Principal Components Analysis on these nodes and modifies the
     random configuration according to that analysis.
     \param fatherNode Node to extend from.
     \param qRand Initial random configuration.
     \param minNb min number of nodes needed to do a PCA
     \param resultCfg pointer to a Config where the new 
     direction to extend to will be stored
  */
  void doPCA(const CkwsNodeShPtr fatherNode, 
	     const CkwsConfig qRand,
	     int minNb,
	     CkwsConfig * resultCfg);


 private:
  KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>) m_weakPtr;
  CkwsRoadmapShPtr m_roadmap;

};



template<class T>
CkwsPlusPCARdmBuilder<T>::~CkwsPlusPCARdmBuilder()
{
}



template<class T>
KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
  CkwsPlusPCARdmBuilder<T>::create(
				   const CkwsRoadmapShPtr &i_roadmap,
				   double i_penetration,
				   const CkwsDistanceShPtr &i_evaluator,
				   const CkwsDiffusionNodePickerShPtr &i_picker, 
				   const CkwsDiffusionShooterShPtr &i_shooter
				   )
{
  cout << "CkwsPlusPCARdmBuilder::create" << endl;
  CkwsPlusPCARdmBuilder<T> * devPtr = new CkwsPlusPCARdmBuilder<T>(i_roadmap,i_evaluator);
  KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>) devShPtr(devPtr);
  KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>) devWkPtr(devShPtr);

  if (devPtr->init(devWkPtr,i_penetration) != KD_OK){
    devShPtr.reset();
  }

  return devShPtr;

}

template<class T>
KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
  CkwsPlusPCARdmBuilder<T>::createCopy(const KIT_SHARED_PTR_CONST(CkwsPlusPCARdmBuilder<T>) &inRdmpBuilder,
				       double i_penetration)
{
  if(inRdmpBuilder != NULL) {
    CkwsPlusPCARdmBuilder<T>* devPtr = new CkwsPlusPCARdmBuilder<T>(*inRdmpBuilder);
    KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>) devShPtr(devPtr);
    KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>) devWkPtr(devShPtr);

    if(devPtr->init(devWkPtr,i_penetration) != KD_OK){
      devShPtr.reset();
    }
    return devShPtr;
  }
  return KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)();
}


template<class T>
CkwsPlusPCARdmBuilder<T>::CkwsPlusPCARdmBuilder(const CkwsRoadmapShPtr& i_roadmap,
						const CkwsDistanceShPtr& i_evaluator)
  : T(i_roadmap,i_evaluator)
{

}


template<class T>
ktStatus CkwsPlusPCARdmBuilder<T>::init(const KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>)& i_weakPtr, 
					double i_penetration)
{
  ktStatus success = T::init(i_weakPtr,i_penetration);

  if (KD_OK == success){
    m_weakPtr = i_weakPtr;
  }
  return success ;
}


template<class T>
CkwsNodeShPtr CkwsPlusPCARdmBuilder<T>::extend(const CkwsNodeShPtr & i_node,
					       const CkwsConfig & i_cfg,
					       CkwsRoadmapBuilder::EDirection i_direction )
{
  unsigned int n = T::roadmap()->device()->countDofs();
  

  double do_project = (double) rand()/(INT_MAX);
  double pb = 0.5;
  
  bool projected = false;
  CkwsConfig newCfg(T::roadmap()->device());

  if(do_project<pb)
    {
      if(i_node->connectedComponent()->countNodes()> n)
	{
	  doPCA(i_node,i_cfg,5*n,&newCfg);
	  projected = true;
	}
    }

  if(projected)
    {
      CkwsNodeShPtr newNode = T::extend(i_node,newCfg,i_direction);
      return newNode;
    }
  else 
    {
      CkwsNodeShPtr newNode = T::extend(i_node,i_cfg,i_direction);
      return newNode;
    }
}



template<class T>
void CkwsPlusPCARdmBuilder<T>::doPCA(const CkwsNodeShPtr fatherNode, 
				     const CkwsConfig qRand,
				     int minNb,
				     CkwsConfig * resultCfg)
{


  int n = T::roadmap()->device()->countDofs();
  vector<double> * tab = new vector<double>[n];       

 /* Breadth-first search to find the neighbours  used in the PCA*/

  map<CkwsNodeShPtr , int> color;
  map<CkwsNodeShPtr , int> distance;
  queue<CkwsNodeShPtr> F;

  int neighbours = 0;
  int d_max = INT_MAX;

  F.push(fatherNode);
  color[fatherNode] = 1;
  distance[fatherNode] = 0;

  while(!F.empty()) {
    CkwsNodeShPtr current = F.front();
    if (distance[current] > d_max)
      break;
    unsigned int i;
    for(i = 0; i< current->countOutEdges();i++){
      CkwsNodeShPtr son = current->outEdge(i)->endNode();
      if ((color[son] != 1)&&(color[son] != 2)){
	color[son] = 1;
	distance[son] = distance[current] +1;
	F.push(son);
      }
    }

    CkwsConfig currentConfig= current->config();
    int j;
    for(j=0;j<n;j++){
      tab[j].push_back(currentConfig.dofValue(j));
    }

    neighbours++;
    if(neighbours > minNb)
      d_max = distance[current];

    F.pop();
    color[current] = 2;
  }
  
  namespace ublas = boost::numeric::ublas ;
  namespace lapack = boost::numeric::bindings::lapack ;
 
  ublas::matrix<double, ublas::column_major> cov (n,n); //covariance matrix 
  double * means = new double[n];

  int i,j,k;
  /* First we compute the estimated mean vector. */
  for(i=0;i<n;i++){
    double meanI = 0;
    for(k=0;k<neighbours;k++){
      meanI += (tab[i][k] - meanI)/ (k +1);
    }
    means[i] = meanI;
  }

  /* We then compute the estimated variances... */
  for(i=0;i<n;i++){
    double variance = 0;
    for(k=0;k<neighbours;k++){
      double delta = tab[i][k] - means[i];
      variance += (delta * delta - variance) / (k+1);
    }
    cov (i,i) = variance *((double) neighbours / (double)(neighbours-1));
  }

  /* ...and the estimated covariances. */
  for(i=0;i<n;i++){
    for(j=0;j<i;j++){
      double covariance = 0;
      for(k=0;k<neighbours;k++){
	double delta1 = tab[i][k] - means[i];
	double delta2 = tab[j][k] - means[j];
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


  delete [] tab;
  delete [] means;



}

typedef  CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder > CkwsPlusPCADiffusingRdmBuilder;
typedef  CkwsPlusPCARdmBuilder<CkwsIPPRdmBuilder> CkwsPlusPCAIPPRdmBuilder;


KIT_POINTER_DEFS(  CkwsPlusPCADiffusingRdmBuilder );
KIT_POINTER_DEFS(  CkwsPlusPCAIPPRdmBuilder );




#endif
