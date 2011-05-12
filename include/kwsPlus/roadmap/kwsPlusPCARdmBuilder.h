/*
  include file for kwsPlusPCARdmBuilder

  Developed by Sebastien Dalibard (LAAS-CNRS)

  Copyright 2011 LAAS-CNRS
*/


#ifndef KWS_PLUS_PCA_ROADMAP_BUILDER
#define KWS_PLUS_PCA_ROADMAP_BUILDER


/*************************************
INCLUDE
**************************************/

#include <vector>
#include <cmath>
#include <queue>
#include <fstream>
#include <algorithm>

#include "KineoWorks2/kwsValidatorDPCollision.h"
#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsDistance.h"
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

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/bindings/lapack/syev.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/ublas/io.hpp>


#define PCA_FREQUENCY 0.5
#define PBT_THRESHOLD 0.1
#define PBT_UPPER_THRESHOLD 2.0
#define MAX_NB_POINTS 0.5


KIT_PREDEF_CLASS( CkwsRoadmap );


template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusPCARdmBuilder; 

class CkwsDiffusingRdmBuilder;
class CkwsIPPRdmBuilder;

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
  */

  static 
    KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
    create(const CkwsRoadmapShPtr& i_roadmap);


  /**
     \brief Copy constructor.
     \param inRdmpBuilder The roadmap builder to copy.
  */
  static 
    KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
    createCopy(const  KIT_SHARED_PTR_CONST(CkwsPlusPCARdmBuilder<T>) &inRdmpBuilder);

  /*
  double getAverageRRT() const ;
  double getAveragePCA() const ;
  int getNbIterationsRRT() const ;
  int getNbIterationsPCA() const ;
  int getNbSuccessfullPCA() const ;
  int getNbSuccessfullRRT() const ;
  int getNbUnfinishedPCA() const ;
  double getAverageNbPoints() const ;
  */


 protected:
  /**
     \brief Constructor
     \param i_roadmap: The roadmap to be built,
  */
  CkwsPlusPCARdmBuilder(const CkwsRoadmapShPtr& i_roadmap);


  /**
     \brief Init function
     \param i_weakPtr The weak pointer to the PCARdmBuilder.
     \return ktStatus KD_OK or KD_ERROR
  */

  ktStatus init(const KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>)& i_weakPtr);


  /**
     \brief Extend function: extends the roadmap to/from a node to/from 
     a random configuration. Here, we first call doPCA before calling the
     standard diffusing extend function with the new random configuration.
     \param i_node Node to extend from/to.
     \param i_cfg Configuration to extend to/from. 
     \param i_direction ROADMAP_TO_NODE or NODE_TO_ROADMAP
     \return The new node of the roadmap (can be NULL)
  */
  CkwsNodeShPtr extend(const CkwsNodeShPtr & i_node,
		       const CkwsConfig & i_cfg,
		       CkwsRoadmapBuilder::EDirection i_direction );

  /**
     \brief doPCA function: first does a Breadth-First Search in the roadmap 
     starting from the node to extend from, when it has got enough nodes
     performs a Principal Components Analysis on these nodes and modifies the
     random configuration according to that analysis.
     \param fatherNode Node to extend from.
     \param qRand Initial random configuration.
     \param resultCfg pointer to a Config where the new 
     direction to extend to will be stored
  */
  ktStatus doPCA(const CkwsNodeShPtr & fatherNode, 
		 const CkwsConfig & qRand,
		 CkwsConfig * resultCfg);

  /**
     \brief recPCA function: computes one step of recursive PCA.
     \param k arriving sample index
     \param means Mean configuration after adding sample k
     \param newPoint new sample to be added to PCA analysis
     \param eigenVectors matrix containing the eigen vectors, will be updated
     \param eigenValues vector containing the eigen values, will be updated

  */
  void recPCA(
	      unsigned int k,
	      boost::numeric::ublas::vector<double> &means,	      
	      boost::numeric::ublas::vector<double> &newPoint,
	      boost::numeric::ublas::matrix<double,  boost::numeric::ublas::column_major> &eigenVectors,
	      boost::numeric::ublas::vector<double> &eigenValues
	      );


  /**
     \brief getDeviceInfo function: stores the information about bounds on dofs of the considered device
  */
  void getDeviceInfo();
  
  void configToNormalizedUblas(CkwsConfig & cfg,
			       boost::numeric::ublas::vector<double> & boundDofVector,
			       boost::numeric::ublas::vector<double> & unboundDofVector);

  double distance2(boost::numeric::ublas::vector<double> & config1,
		   boost::numeric::ublas::vector<double> & config2) ;

  std::vector<double> ublasToStd(boost::numeric::ublas::vector<double> & in_v);
  
  double pcaPrecision(std::vector<double> vap,
		      double r,
		      int p
		      );
  

 private:
  KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>) m_weakPtr;
  CkwsRoadmapShPtr m_roadmap;

  unsigned int n_dofs;
  std::vector<int> unboundDofs;
  std::vector<int> boundDofs;
  std::map<int,double> dofWidth;
  unsigned int nbNeighbours;

  //DEBUG 
  /*
    unsigned int nbRecPCA;
    double averageRRT;
    double averagePCA;
    int nbIterationsRRT;
    int nbIterationsPCA;
    int nbSuccessfullPCA;
    int nbSuccessfullRRT;
    int nbUnfinishedPCA;
    double averageNbPoints;
  */
  //std::fstream out_fstream;
  //END_DEBUG

};



template<class T>
CkwsPlusPCARdmBuilder<T>::~CkwsPlusPCARdmBuilder()
{
}



template<class T>
KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
CkwsPlusPCARdmBuilder<T>::create(
				 const CkwsRoadmapShPtr &i_roadmap
				 )
{
  CkwsPlusPCARdmBuilder<T> * pcaRdmBuilderPtr = new CkwsPlusPCARdmBuilder<T>(i_roadmap);
  KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>) pcaRdmBuilderShPtr(pcaRdmBuilderPtr);
  KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>) pcaRdmBuilderWkPtr(pcaRdmBuilderShPtr);

  if (pcaRdmBuilderPtr->init(pcaRdmBuilderWkPtr) != KD_OK){
    pcaRdmBuilderShPtr.reset();
  }
  pcaRdmBuilderShPtr->getDeviceInfo();
  
  return pcaRdmBuilderShPtr;
  
}

template<class T>
KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)
CkwsPlusPCARdmBuilder<T>::createCopy(const KIT_SHARED_PTR_CONST(CkwsPlusPCARdmBuilder<T>) &inRdmpBuilder)
{
  if(inRdmpBuilder != NULL) {
    CkwsPlusPCARdmBuilder<T>* pcaRdmBuilderPtr = new CkwsPlusPCARdmBuilder<T>(*inRdmpBuilder);
    KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>) pcaRdmBuilderShPtr(pcaRdmBuilderPtr);
    KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>) pcaRdmBuilderWkPtr(pcaRdmBuilderShPtr);
    
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
  return KIT_SHARED_PTR(CkwsPlusPCARdmBuilder<T>)();
}


template<class T>
CkwsPlusPCARdmBuilder<T>::CkwsPlusPCARdmBuilder(const CkwsRoadmapShPtr& i_roadmap)
: T(i_roadmap)
{

}


template<class T>
ktStatus CkwsPlusPCARdmBuilder<T>::init(const KIT_WEAK_PTR(CkwsPlusPCARdmBuilder<T>)& i_weakPtr)
{
  ktStatus success = T::init(i_weakPtr);

  if (KD_OK == success){
    m_weakPtr = i_weakPtr;
  }
  return success ;
}


template<class T>
void CkwsPlusPCARdmBuilder<T>::getDeviceInfo()
{ 
  double currentWidth;

  //DEBUG
  /*
  averageRRT = 0;
  averagePCA = 0;
  nbIterationsRRT = 0;
  nbIterationsPCA = 0;
  nbSuccessfullPCA = 0;
  nbSuccessfullRRT = 0;
  averageNbPoints = 0;
  nbUnfinishedPCA = 0;
  */
  //End DEBUG

  unboundDofs.clear();
  boundDofs.clear();
  dofWidth.clear();

  CkwsDofShPtr currentDof;
  CkwsDeviceShPtr device = T::roadmap()->device();
  n_dofs = device->countDofs();
  

  //  nbRecPCA = 0;

  unsigned int i;

  for (i = 0; i < n_dofs ; i++)
    {
      currentDof = device->dof(i);
      if ( !currentDof->isLocked() ) 
	{
          if ( currentDof->isBounded() )
            {
              currentWidth =  currentDof->vmax() - currentDof->vmin() ;
              if ( currentWidth )
		{
		  boundDofs.push_back(i);
		  dofWidth[i] = currentWidth ;
		}
            }
          else if ( currentDof->isRevolute() )
            {
              boundDofs.push_back(i);
              dofWidth[i] = 2*M_PI;
            }
          else
            unboundDofs.push_back(i);
	}
    }

  nbNeighbours = std::max(boundDofs.size(),unboundDofs.size()) + 1 ;
}


template<class T>
CkwsNodeShPtr CkwsPlusPCARdmBuilder<T>::extend(const CkwsNodeShPtr & i_node,
					       const CkwsConfig & i_cfg,
					       CkwsRoadmapBuilder::EDirection i_direction )
{
  double do_project = (double) rand()/(RAND_MAX);
  bool projected = false;
  CkwsConfig newCfg(T::roadmap()->device());
  CkwsNodeShPtr newNode;

  if(do_project< PCA_FREQUENCY)
    {
      if(i_node->connectedComponent()->countNodes()> n_dofs)
        {
          if ( doPCA(i_node,i_cfg,&newCfg) == KD_OK)
	    projected = true;
	  /*
	  else
	    nbUnfinishedPCA++;
	  */
        }
    }
  
  if(projected){
    //nbIterationsPCA++ ;

    newNode = T::extend(i_node,newCfg,i_direction);
    
    //DEBUG
    /*
    if(newNode) {
      nbSuccessfullPCA++;
      averagePCA += ( T::distance()->distance(i_node->config(),newNode->config()) - averagePCA )/ nbSuccessfullPCA;
      //std::cout << "projection worked" << std::endl;
    }
    */
    //END_DEBUG

  }
  else {   
    //nbIterationsRRT++;

    newNode = T::extend(i_node,i_cfg,i_direction);

    //DEBUG
    /*
    if (newNode) {
      nbSuccessfullRRT++;
      averageRRT += ( T::distance()->distance(i_node->config(),newNode->config()) - averageRRT )/ nbSuccessfullRRT ;
    }
    */
    //END_DEBUG

  }  

  //DEBUG
  /*
    std::cout << " -----------------------------" << std::endl 
    << "Reussite RRT : " << nbSuccessfullRRT  << " / " << nbIterationsRRT 
    <<  " , Distance moyenne : " << averageRRT  
    << std::endl 
    << "Reussite PCA : " << nbSuccessfullPCA   << " / " << nbIterationsPCA
    << " , Distance moyenne : " << averagePCA
    << std::endl
    << "PCA ne converge pas : " << nbUnfinishedPCA << std::endl
    << "Nombre moyen de points utilises : " << averageNbPoints 
    << std::endl <<  " -----------------------------" << std::endl ;
  */
  //END_DEBUG


  return newNode;
}

template<class T>
std::vector<double> CkwsPlusPCARdmBuilder<T>::ublasToStd(
							 boost::numeric::ublas::vector<double> & in_v
							 )
{
  unsigned int i ;
  std::vector<double> out_v;
  for ( i = 0 ; i<in_v.size() ; i++)
    out_v.push_back(in_v(i));
  return out_v;
}

template<class T>
void CkwsPlusPCARdmBuilder<T>::configToNormalizedUblas(
						       CkwsConfig & cfg,
						       boost::numeric::ublas::vector<double> & boundDofVector,
						       boost::numeric::ublas::vector<double> & unboundDofVector
						       )
{
  unsigned int i ;

  for( i = 0 ; i < boundDofs.size() ; i++ )    
    boundDofVector ( i ) = cfg.dofValue ( boundDofs[i] ) / dofWidth[ boundDofs[i] ];
        
  for( i = 0 ; i < unboundDofs.size() ; i++ )
    unboundDofVector ( i ) = cfg.dofValue( unboundDofs[i] );
        
  return ;
}


template<class T>
double CkwsPlusPCARdmBuilder<T>::distance2(boost::numeric::ublas::vector<double> & config1,
					   boost::numeric::ublas::vector<double> & config2) 
{
  double result = 0;
  unsigned int n = config1.size();
  unsigned int i;
  if (config2.size() != n)
    return result;

  for(i=0;i<n;i++)
    result+=pow(config1(i) -config2(i),2);
  
  return result;
}

template<class T>
double CkwsPlusPCARdmBuilder<T>::pcaPrecision(
					      std::vector<double> vap,
					      double r,
					      int p
					      )
{
  unsigned int i;
  double currentPrecision;
  double bestPrecision = std::numeric_limits<double>::max();

  //DEBUG
  //unsigned int i_best = 0;
  //END DEBUG


  if(vap.size() < 2)
    bestPrecision = 0;
  
  std::sort(vap.begin(),vap.end());
  
  for( i = 1 ; i <vap.size()  ; i++ )
    {
      if (vap[i] - vap[i-1])
        {
          currentPrecision = 1./( sqrt ( (double) i) * (vap[i] - vap[i-1]));

	  //DEBUG
	  //if(bestPrecision > currentPrecision) i_best = i;
	  //END DEBUG

          bestPrecision = (currentPrecision < bestPrecision) ? currentPrecision : bestPrecision ;
	}
    }

  //DEBUG
  /*
    if(bestPrecision) {

    double sum = 0;
    for( i = 0 ; i < vap.size() ; i++) sum += vap[i] ;
    
    double partial_sum = 0;
    for( i = 0 ; i < i_best ; i++) partial_sum += vap[i] ;
    
    double residuel = partial_sum / sum;
       
    out_fstream << i_best << "\t" 
    << (bestPrecision*r/((double) sqrt(p))) << "\t"
    << residuel << std::endl;
    
    }
  */
  //END_DEBUG


  return (bestPrecision*r/((double) sqrt(p)));


}


template<class T>
ktStatus CkwsPlusPCARdmBuilder<T>::doPCA(const CkwsNodeShPtr & fatherNode, 
					 const CkwsConfig & qRand,
					 CkwsConfig * resultCfg)
{

  //DEBUG
  /*
    char filename[256];
    sprintf(filename,"/home/sdalibar/devel/results/pcaIteration_%d.txt",nbIterationsPCA) ;
    out_fstream.open( filename ,std::fstream::out );

    out_fstream << "Points\tDimension\tPrecision\tResidual Variance" << std::endl;
  */
  //END_DEBUG


  namespace ublas = boost::numeric::ublas ;
  namespace lapack = boost::numeric::bindings::lapack ;

  unsigned int i,j,k;


  //Mean vector for bounded dofs
  ublas::vector<double> boundMeans (boundDofs.size());

  //Mean vector for unbounded dofs
  ublas::vector<double> unboundMeans (unboundDofs.size());
  
  //Covariance matrix for bounded dofs
  ublas::matrix<double, ublas::column_major> unboundCov (unboundDofs.size(),unboundDofs.size()); 

  //Covariance matrix for unbounded dofs
  ublas::matrix<double, ublas::column_major> boundCov (boundDofs.size(),boundDofs.size()); 

  //Eigen values vector for unbounded dofs
  ublas::vector<double> unboundEigenValues (unboundDofs.size());

  //Eigen values vector for bounded dofs
  ublas::vector<double> boundEigenValues (boundDofs.size());

  //Vector of normalized sampled points 
  std::vector< ublas::vector<double> > boundPoints;
  std::vector< ublas::vector<double> > unboundPoints;


  /* Breadth-first search to find the neighbours  used in the standard PCA*/

  std::map<CkwsNodeShPtr , int> color;
  std::map<CkwsNodeShPtr , int> distance;
  std::queue<CkwsNodeShPtr> F;

  unsigned int neighbours = 0;
  unsigned int argMin_precision;
  double bestPrecision;
  double rBound_max = 0;
  double rUnbound_max = 0;
  double current_rBound , current_rUnbound ;

  CkwsConfig fatherConfig = fatherNode->config();

  F.push(fatherNode);
  color[fatherNode] = 1;
  distance[fatherNode] = 0;


  while ( (!F.empty()) && 
	  (neighbours <=  std::max( nbNeighbours ,(unsigned int) 3) ) ) 
    {
      CkwsNodeShPtr current = F.front();
      
      for(i = 0; i< current->countOutEdges();i++)
        {
          CkwsNodeShPtr son = current->outEdge(i)->endNode();
          if ((color[son] != 1)&&(color[son] != 2)){
            color[son] = 1;
            distance[son] = distance[current] +1;
            F.push(son);
          }
        }
      
      CkwsConfig currentConfig= current->config();

      ublas::vector<double> boundDofVector( boundDofs.size() ); 
      ublas::vector<double> unboundDofVector( unboundDofs.size() );
      configToNormalizedUblas ( currentConfig, boundDofVector , unboundDofVector );
	
      boundPoints.push_back ( boundDofVector );
      unboundPoints.push_back ( unboundDofVector );
	
      current_rBound = distance2(boundPoints[0] , boundDofVector ); 
      current_rUnbound = distance2(unboundPoints[0] ,unboundDofVector ); 

      rBound_max   = (current_rBound > rBound_max) ? current_rBound : rBound_max ;             
      rUnbound_max = (current_rUnbound > rUnbound_max) ? current_rUnbound : rUnbound_max ;     
	      
      neighbours++;
      F.pop();
      color[current] = 2;
    }
  
  /* Computing the estimated mean vector. */
  boundMeans   = ublas::zero_vector<double>::zero_vector(  boundDofs.size()  );
  unboundMeans = ublas::zero_vector<double>::zero_vector( unboundDofs.size() );
    
  for(k=0;k<neighbours;k++){
    boundMeans    += (  boundPoints[k] -   boundMeans)/ (k +1);
    unboundMeans  += (unboundPoints[k] - unboundMeans)/ (k +1);
  }
  /* We then compute the estimated variances... */
  
  /* For the bounded dofs */
  for(i=0;i<boundDofs.size();i++)
    {
      double variance = 0;
      for(k=0;k<neighbours;k++){
        double delta = boundPoints[k](i)  - boundMeans(i);
	variance += (delta * delta - variance) / (k+1);
      }
      boundCov (i,i) = variance *((double) neighbours / (double)(neighbours-1));
    }

  /* For the unbounded dofs */
  for(i=0;i<unboundDofs.size();i++)
    {
      double variance = 0;
      for(k=0;k<neighbours;k++){
        double delta = unboundPoints[k](i) - unboundMeans(i);
        variance += (delta * delta - variance) / (k+1);
      }
      unboundCov (i,i) = variance *((double) neighbours / (double)(neighbours-1));
    }


  /* ...and the estimated covariances. */

  /* For the bounded dofs */
  for(i=0;i<boundDofs.size();i++)
    {
      for(j=0;j<i;j++)
	{
	  double covariance = 0;
	  for(k=0;k<neighbours;k++){
	    double delta1 = boundPoints[k](i) - boundMeans(i);
	    double delta2 = boundPoints[k](j) - boundMeans(j);
	    covariance += (delta1 * delta2 - covariance) / (k+1);
	  }
	  covariance *= (double) neighbours / (double) (neighbours -1);
	  boundCov (i,j) = covariance;
	  boundCov (j,i) = covariance;
	}
    }

  /* For the unbounded dofs */
  for(i=0;i<unboundDofs.size();i++)
    {
      for(j=0;j<i;j++)
	{
	  double covariance = 0;
	  for(k=0;k<neighbours;k++){
	    double delta1 = unboundPoints[k](i) - unboundMeans(i);
	    double delta2 = unboundPoints[k](j) - unboundMeans(j);
	    covariance += (delta1 * delta2 - covariance) / (k+1);
	  }
	  covariance *= (double) neighbours / (double) (neighbours -1);
	  unboundCov (i,j) = covariance;
	  unboundCov (j,i) = covariance;
	}
    }
  
  if (boundDofs.size()) {
    lapack::syev('V',
		 'L',
		 boundCov,
		 boundEigenValues,
		 lapack::optimal_workspace());
  }  

  if (unboundDofs.size()) {
    lapack::syev('V',
		 'L',
		 unboundCov,
		 unboundEigenValues,
		 lapack::optimal_workspace());
  }

  //DEBUG
  /*
    std::cout << "Classic PCA Computation " << std::endl;
    std::cout << "Bound Eigenvectors : " << boundCov << std::endl;
    std::cout << "Bound Eigenvalues : " << boundEigenValues << std::endl;
    std::cout << "UnBound Eigenvectors : " << unboundCov << std::endl;
    std::cout << "UnBound Eigenvalues : " <<  unboundEigenValues << std::endl;


    out_fstream << neighbours << "\t";
  */
  //END_DEBUG
 

  std::vector<double> std_boundEigenValues = ublasToStd(boundEigenValues);
  std::vector<double> std_unboundEigenValues = ublasToStd(unboundEigenValues);
  
  double bound_pca_prec = pcaPrecision(std_boundEigenValues,  rBound_max ,neighbours);
  double unbound_pca_prec = pcaPrecision(std_unboundEigenValues,rUnbound_max  ,neighbours);
  
  double precision = std::max ( bound_pca_prec,  unbound_pca_prec );
  bestPrecision = precision;
  argMin_precision = neighbours;

  if (std::isnan(precision) || std::isinf(precision))
    return KD_ERROR;
  
  /*Computing recursive PCA until the identified subspace converges */

  while ( ( precision > PBT_THRESHOLD ) 
	  && ( !F.empty() ) 
	  && ( !std::isnan(precision) ) 
	  && ( !std::isinf(precision) ) 
	  && ( precision < PBT_UPPER_THRESHOLD )
	  && ( argMin_precision > neighbours * MAX_NB_POINTS  ) )
    {

      //DEBUG

      /*
	std::cout << precision << "\t" ;
	std::cout << "Rec PCA Computation: ---------------- \n  bounded Dofs:\n " ;
	for (i=0;i<boundDofs.size();i++){
	std::cout << "Eigen Value:\t" << boundEigenValues(i) << std::endl;
	std::cout << "Eigen Vector: \t" ;
	for (j=0;j<boundDofs.size();j++) std::cout << boundCov(j,i) * dofWidth[ boundDofs[j] ] << "\t";
	std::cout << std::endl;
	}
	std::cout << "Rec PCA Computation: ---------------- \n  unbounded Dofs:\n " ;
	for (i=0;i<unboundDofs.size();i++){
	std::cout << "Eigen Value:\t" << unboundEigenValues(i) << std::endl;
	std::cout << "Eigen Vector: \t" ;
	for (j=0;j<unboundDofs.size();j++) std::cout << unboundCov(j,i)  << "\t";
	std::cout << std::endl;
	}
      */

      //END_DEBUG


      //finding next neighbour
      CkwsNodeShPtr current = F.front();
      
      for(i = 0; i< current->countOutEdges();i++)
	{
	  CkwsNodeShPtr son = current->outEdge(i)->endNode();
	  if ((color[son] != 1)&&(color[son] != 2)){
	    color[son] = 1;
	    distance[son] = distance[current] +1;
	    F.push(son);
	  }
	}
      
      CkwsConfig currentConfig= current->config();

      ublas::vector<double> boundDofVector( boundDofs.size() ); 
      ublas::vector<double> unboundDofVector( unboundDofs.size() );
      configToNormalizedUblas ( currentConfig,boundDofVector , unboundDofVector );
      
      boundPoints.push_back ( boundDofVector );
      unboundPoints.push_back ( unboundDofVector );

      neighbours++;

      //updating mean vector
    
      boundMeans    += (  boundPoints[neighbours - 1] -   boundMeans)/ neighbours;
      unboundMeans  += (unboundPoints[neighbours - 1] - unboundMeans)/ neighbours;
  
      current_rBound = distance2(boundPoints[0] , boundDofVector ); 
      current_rUnbound = distance2(unboundPoints[0] ,unboundDofVector ); 
      
      rBound_max   = (current_rBound > rBound_max) ? current_rBound : rBound_max ;             
      rUnbound_max = (current_rUnbound > rUnbound_max) ? current_rUnbound : rUnbound_max ;     
			  
      //updating recursive PCA  computation

      if ( unboundDofs.size() )
	recPCA( neighbours ,unboundMeans , unboundDofVector , unboundCov ,unboundEigenValues );

      if ( boundDofs.size() )
	recPCA( neighbours ,boundMeans , boundDofVector , boundCov , boundEigenValues);

      std_boundEigenValues = ublasToStd(boundEigenValues);
      std_unboundEigenValues = ublasToStd(unboundEigenValues);
      
      //DEBUG
      //      out_fstream << neighbours << "\t";
      //END_DEBUG

      bound_pca_prec = pcaPrecision(std_boundEigenValues, rBound_max ,neighbours);
      unbound_pca_prec = pcaPrecision(std_unboundEigenValues,rUnbound_max ,neighbours);
      precision = std::max ( bound_pca_prec,  unbound_pca_prec );

      if (precision <= bestPrecision) 
	{
	  bestPrecision = precision;
	  argMin_precision = neighbours;
	}


      F.pop();
      color[current] = 2;      
    }


  if ( (precision >  PBT_THRESHOLD) || std::isnan(precision) || std::isinf(precision) ) {

    //DEBUG
    /*
      std::cout << std::endl << "Desired Precision not reached,  reached Precision: " << precision << std::endl;
    */
    //    out_fstream.close();
    //END_DEBUG
    return KD_ERROR;
  }

  //DEBUG
  /*
    std::cout << std::endl << "Reached Precision: " << precision << std::endl;
  */
  //END_DEBUG

  double boundMax = 0;
  double unboundMax = 0;

  for(i=0;i<boundDofs.size();i++){
    boundMax   = ( boundEigenValues(i) > boundMax ) ?  boundEigenValues(i) : boundMax ;   
  }
  
  for(i=0;i<unboundDofs.size();i++){
    unboundMax   = ( unboundEigenValues(i) > unboundMax ) ?  unboundEigenValues(i) : unboundMax ;   
  }

  
  CkwsConfig qNear = fatherNode->config();
  ublas::vector<double> randomVector (n_dofs);
  ublas::vector<double> newRandomVector(n_dofs);
  
  for(i = 0; i<n_dofs ; i++)
    {
      randomVector(i) = qRand.dofValue(i) - qNear.dofValue(i) ;
      newRandomVector(i) = 0.;
    }



  //DEBUG
  /*
    std::cout << "doPCA -- about to project" << std::endl;
    std::cout << "bound Dofs" << std::endl;
  */
  //END_DEBUG

  for(i = 0;i<boundDofs.size();i++)
    {

      //DEBUG
      /*
	std::cout << "Eigen Value: " << boundEigenValues(i) << std::endl;
      */
      //END_DEBUG


      double coeff = boundEigenValues(i) / boundMax;
      ublas::vector<double> eigenVector (n_dofs); 
      eigenVector = ublas::zero_vector<double>::zero_vector(n_dofs);
      for(j=0;j<boundDofs.size();j++)
	{
	  eigenVector(boundDofs[j]) = boundCov(j,i) * dofWidth[ boundDofs[j] ] ;
	}


      //DEBUG
      /*
	std::cout << "Eigen Vector: ";
	std::cout << eigenVector   << std::endl;
      */
      //END_DEBUG   

      double alpha = ublas::inner_prod(randomVector, eigenVector)/( ublas::norm_2(eigenVector) );
      newRandomVector += alpha*coeff*eigenVector;
    }

  //DEBUG
  /*
    std::cout << "unbound Dofs" << std::endl;
  */
  //END_DEBUG

  for(i = 0;i<unboundDofs.size();i++)
    { 

      //DEBUG
      /*
	std::cout << "Eigen Value: " << unboundEigenValues(i) << std::endl;
      */
      //END_DEBUG

      double coeff = unboundEigenValues(i) / unboundMax;
      ublas::vector<double> eigenVector (n_dofs);
      eigenVector = ublas::zero_vector<double>::zero_vector(n_dofs);
      for(j=0;j<unboundDofs.size();j++)
	{
	  eigenVector(unboundDofs[j]) = unboundCov(j,i);
	}

      //DEBUG
      /*
	std::cout << "Eigen Vector: ";
	std::cout << eigenVector  << std::endl;
      */
      //END_DEBUG
    
      double alpha = ublas::inner_prod(randomVector, eigenVector)/( ublas::norm_2(eigenVector) );
      newRandomVector += alpha*coeff*eigenVector;
    }

  newRandomVector *= ublas::norm_2(randomVector) / ublas::norm_2(newRandomVector);

  for(i=0;i<n_dofs;i++){
    double dofI = qNear.dofValue(i) + newRandomVector(i);
    resultCfg->dofValue(i,dofI);
  }


  

  //DEBUG
  //averageNbPoints += (neighbours - averageNbPoints)/(nbIterationsPCA + 1 ) ;
  //out_fstream.close();
  //nbRecPCA += (neighbours - nbNeighbours)/(nbIterationsPCA + 1);
  //END_DEBUG


  return KD_OK;

}


template<class T>
void CkwsPlusPCARdmBuilder<T>::recPCA(
				      unsigned int k,
				      boost::numeric::ublas::vector<double> &means,
				      boost::numeric::ublas::vector<double> &newPoint,
				      boost::numeric::ublas::matrix<double,  boost::numeric::ublas::column_major> &eigenVectors,
				      boost::numeric::ublas::vector<double> &eigenValues
				      )
{
  namespace ublas = boost::numeric::ublas ;
  unsigned int n = eigenValues.size();

  static ublas::vector<double> x;   //Q_{k-1}^T (x_k - m_k)

  double coeffA = 1 / (double) (k) + 1 / ( (double) k * (k-1.) ) ;


  static ublas::matrix<double> qT;
  qT = ublas::trans(eigenVectors) ;
  x  = ublas::prod(qT,newPoint - means);


  static ublas::matrix<double> pV(n,n);
  static ublas::vector<double> pL(n);

  static ublas::vector<double> D(n);
  static ublas::matrix<double> V(n,n);
  static ublas::identity_matrix<double> I (n);

  static ublas::vector<double> eigenVector(n);

  unsigned int i,j;
  
  for( i=0 ; i<n ; i++) {
    pL(i) = coeffA * pow( x(i) ,2);
    pV(i,i) = 0;
  }

  for( i=0 ; i<n ; i++ ) {
    for ( j=0 ; j<i ; j++) {
      pV(i,j) = (coeffA * x(i) * x(j) )/ 
	( (k-1.)/( (double) k ) * eigenValues(j) +  pL(j) -  (k-1.)/( (double) k ) * eigenValues(i) - pL(i) );
      pV(j,i) = -pV(i,j);
    }
  }

  D = (k-1.)/( (double) k ) * eigenValues + pL;
  V = I + pV;

  eigenVectors = ublas::prod(eigenVectors , V);
  eigenValues  = D ;

  //Renormalizing the eigen vector and eigen value estimates
  double t;

  for( i=0 ; i<n ; i++)
    {
      for ( j=0 ; j<n ; j++)
	{
	  eigenVector(j) = eigenVectors(j,i);
	}
      t = ublas::norm_2(eigenVector) ;
      
      for ( j=0 ; j<n ; j++)
	{
	  eigenVectors(j,i) /= t;
	}    
      eigenValues ( i ) *= pow(t,2); 
    }
}

/*
template<class T>
double CkwsPlusPCARdmBuilder<T>::getAverageRRT() const
{
  return averageRRT;
}

template<class T>
double CkwsPlusPCARdmBuilder<T>::getAveragePCA() const
{
  return averagePCA;
}

template<class T>
int CkwsPlusPCARdmBuilder<T>::getNbIterationsRRT() const 
{
  return nbIterationsRRT;
}

template<class T>
int CkwsPlusPCARdmBuilder<T>::getNbIterationsPCA() const 
{
  return nbIterationsPCA;
}

template<class T>
int CkwsPlusPCARdmBuilder<T>::getNbSuccessfullPCA() const 
{
  return nbSuccessfullPCA;
}

template<class T>
int CkwsPlusPCARdmBuilder<T>::getNbSuccessfullRRT() const 
{
  return nbSuccessfullRRT;
}

template<class T>
int CkwsPlusPCARdmBuilder<T>::getNbUnfinishedPCA() const 
{
  return nbUnfinishedPCA;
}

template<class T>
double CkwsPlusPCARdmBuilder<T>::getAverageNbPoints() const 
{
  return averageNbPoints;
}
*/



typedef  CkwsPlusPCARdmBuilder<CkwsDiffusingRdmBuilder > CkwsPlusPCADiffusingRdmBuilder;
typedef  CkwsPlusPCARdmBuilder<CkwsIPPRdmBuilder> CkwsPlusPCAIPPRdmBuilder;


KIT_POINTER_DEFS(  CkwsPlusPCADiffusingRdmBuilder );
KIT_POINTER_DEFS(  CkwsPlusPCAIPPRdmBuilder );

/**
   @}
*/

#endif
