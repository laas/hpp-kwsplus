/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
  Mathieu Poirier (LAAS-CNRS)
 
*/


#ifndef  __REEDS_SHEPP_DP_H
#define __REEDS_SHEPP_DP_H

#warning "deprecated header file. Please include kwsPlus/directPath/reedsSheppDirectPath.h instead."

/*************************************
            INCLUDE
**************************************/


#include "kwsPlus/directPath/kwsPlusDirectPath.h"

#include <iostream>
//#include <KineoWorks2/kwsDirectPath.h>
#include <KineoWorks2/kwsPath.h>
#include <KineoWorks2/kwsSteeringMethod.h>
#include <KineoWorks2/kwsConfig.h>



/**
   \addtogroup RS
   @{
*/

/*************************************
CONSTANTE
**************************************/



/// type For the R&S curve
enum ERsCurveType {
  RS_ALL = 0 ,
  //RS_NO_CUSP = -1,
  //RS_WITH_CUSP = -2,
  //RS_DUBINS = -3
} ;

///Type for the direction
enum EDirType {
  RIGHT = 1,
  LEFT = 2,
  STRAIGHT = 3
} ;

///Type for the direction
enum EConfigType {
  FREEFLYER = 1,
  PLAN= 2,
} ;

/*************************************
STRUCTURES
**************************************/

/**
   \brief Data curve structure for Reeds and Shepp Curve
   \param type : = RIGHT, LEFT or STRAIGHT
   \param cd : Start configuration for the RS curve 
   \param cf : End configuration for the RS curve 
   \param centre_x : center of the circle 
   \param centre_y : on x and y , else 0
   \param r : radius of the circle
   \param sens : = 1/-1 for Forward/Backward
   \param val : value of the curve partition
   \param valid : generaly TRUE when no collision on the trajectory partition
*/
typedef struct TrsCurve
{
  EDirType type;
  CkwsConfigShPtr cd;
  CkwsConfigShPtr cf;
  double centre_x;
  double centre_y;
  double r;
  int sens;
  double val;
  int  valid;
}
  TrsCurve ;

KIT_PREDEF_CLASS( CreedsSheppDirectPath );

// ==============================================================================
//
//  CLASS CreedsSheppDirectPath
//
// ==============================================================================

/**
   \brief Direct path for Reeds and Shepp steering method
*/
class CreedsSheppDirectPath : public CkwsPlusDirectPath
{

public :

  /**
     \brief Destructor 
  */
  ~CreedsSheppDirectPath() ;

  /**
     \brief  Create a new instance of a Reeds and Shepp Direct Path
     \param  inStart : the start configuration
     \param  inEnd 	: the end configuration
     \param  inSteeringMethod 	: shared pointer to the instance of the steering method that created the path
     \param inRadius : the radius used to compute the RS curve
     \param inType : the type of RS to Compute
     \return shared pointer to a newly created DirectPath
  */
  static CreedsSheppDirectPathShPtr create(const CkwsConfig &inStart,const CkwsConfig &inEnd,
					   const CkwsSteeringMethodShPtr &inSteeringMethod , double inRadius, ERsCurveType inType=RS_ALL);

  /**
     \brief  Creates by copy a new instance of aFlic pivoting direct path
     \param inPivotingDP : a shared pointer to hppCmpPivoting that already exist
     \return shared pointer to a newly created DirectPath
  */
  static CreedsSheppDirectPathShPtr createCopy (const CreedsSheppDirectPathConstShPtr &inPivotingDP) ;


  /**
     \brief Returns a shared pointer to a newly allocated copy of the path.
     Note:
     If you want to avoid downcasting the result back to the subclass
     of the parameter, use the appropriate create() method of the subclass
     so as to create a cloned copy of the object while preserving its type.
     \return shared pointer to a newly allocated copy
  */
  virtual CkwsAbstractPathShPtr clone() const;


  /**
     \brief get the internal info Vector of RS
     \return attRsCurveVector
   */
  std::vector< TrsCurve > getInfoRsCurveVector()  ;

protected :

  /// the type of RS curve to compute
  ERsCurveType attTypeCurve ;
  /// the type of the Config FREEFLYER or PLAN
  EConfigType  attTypeConfig ;
  /// vector of all the RS curve partition
  std::vector<TrsCurve>  attRsCurveVector ;
  /// weak pointer to itself
  CreedsSheppDirectPathWkPtr attWeakPtr ;


  /**
     \brief  Constructor (non-default)
     \param inStartCfg : a start configuration
     \param inEndCfg   : a end configuration
     \param inSteeringMethod : a steering Method associed
     \param inType : the type of RS Curve to compute
  */
  CreedsSheppDirectPath(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg,
			const CkwsSteeringMethodShPtr &inSteeringMethod, const ERsCurveType inType);
  /**
     \brief  Constructor copy (non-default)
     \param inDirectPath : a CkwsdirectPath to copy
  */
  CreedsSheppDirectPath(const CreedsSheppDirectPath &inDirectPath);

  /**
     \brief  initialise the weak pointer
     \param inweakPTR : CreedsSheppDirectPathWkPtr
     \return KD_OK or KD_ERROR
  */
  ktStatus init(const CreedsSheppDirectPathWkPtr &inweakPTR) ;

  /**
     \brief  Returns whether the path is oriented.
  */
  virtual bool isOriented () const ;

  /**
     \brief   Returns the parameter range of the direct path at the time it was built.
     \return initial parameter range of the direct path
  */
  virtual double computePrivateLength() const ;

  /**
     \brief   interpolates between m_start and m_end, which are the start configuration
     and the goal configuration the direct path was initially built with.
     \param s 	: private distance this value ranges from zero to privateLength().
     \param outCfg  : configuration on direct path
  */
  virtual void interpolate(double s, CkwsConfig & outCfg) const ;

  /**
     \brief   TODO : not implemented yet :  Return an overestimate of the absolute value of the derivatives param of direct path between two positions on the path.
     Between inFrom and inTo, the absolute value of the i-th dof doesn't vary more than outDerivative[i] * (inTo - inFrom)
     \param inFrom  lower bound of interval.
     \param inTo  upper bound of interval.
     \param outVectorDeriv : On return a vector that contains as many elements as there are dofs in the direct path
  */
  virtual  void maxAbsoluteDerivative(double inFrom, double inTo,
				      std::vector< double > & outVectorDeriv) const;


  /**
     \brief compute an Reeds&Shepp curve between 2 configurations and save it in attRsCurveVector
     \param inStartCfg  the RS start configuration 
     \param inEndCfg the RS End configuration
     \param inRadius the RS Radius 
     \return KD_OK or KD_ERROR
  */

  ktStatus computeRSCurve(const CkwsConfig  inStartCfg , const CkwsConfig inEndCfg , const double inRadius) ;

  /**
     \brief find the number of the RS curve partition in the Vector in which the parameter is.
     \param param the distance from the startConfig
     \return return the number of the RS curve partition in the Vector 
  */
  unsigned int findRsCurveNumInVector(double &param) const ;

  /**
     \brief interpolate the kwsConfig in a selected RSCurse at a selected Distance
     \param u the distance from the startConfig 
     \param outCfg the kwsConfig interpolate
  */
  void kwsConfigAtLengthParam(double u, CkwsConfig &outCfg)const ;

        
  /**
     \name Tools Functions
     @{
  */
  double mod2pi(double a) const ;
  double angleLimitPi(double a) const ;
  double angleBetween2Vector( CkitVect3 startVect , CkitVect3 endVect) ;

  /**
     \brief BEBUG fct TO DELETE
  */
  void printDebug(std::vector<TrsCurve> RSvector) ;
  /**
     \brief BEBUG fct TO DELETE
  */
  void printTrsCurve( const TrsCurve inCurve) ;

  /**
     @}
  */
        
private :
  /**
     \brief compute the liste of C S | function of the RS curve and put in attRsCurveVector
     \param num :  the number of the curve (between 1- 48)
     \param t  : distance value of the first partition of the curve
     \param u : distance value of the second partition of the curve
     \param v : distance value of the ieme partition of the curve
     \param r : radius of the curve 
     \param conf : a configuration to start
     \return KD_OK or KD_ERROR
  */
  ktStatus computeRsCurveVector(int num , double t, double u, double v , double r, CkwsConfig &conf ) ;

  /**
     \brief build a specific RS partition and add it in the attRsCurveVector
     \param r : radius of the curve 
     \param ty  : = 1/2/3 for right/left/straigth
     \param se : = 1/-1 for forward/backward
     \param val : distance value of the  partition's curve
     \param inNewStart - a configuration to start
  */
  void addRsCurveToVector(double r, int ty , int se , double val , CkwsConfig &inNewStart ) ;

  /**
     \brief re-compute the length of a partition between its startCfg (cd) and endCfg (cf) 
     \param curCurve the Partition of the curve that you want to re-compute.
   */
  void computeNewVal(TrsCurve &curCurve) ;

  /**
     \name Calculation Function
     @{
  */


  /**
     \brief compute the minimun distance between 2 configurations. 
     note : It check the 48 cases of R&S to find the min structure CCSCC||
     \param c1 :  a RS configuration
     \param c2 : a RS configuration
     \param radius : the RS Radius 
     \param numero  : the number of the curve in the table
     \param t_r : distance value of the first partition of the curve
     \param u_r : distance value of the second partition of the curve
     \param v_r : distance value of the ieme partition of the curve
     \return length between c1 and c2
  */
  double reed_shepp(CkwsConfig &c1 , CkwsConfig &c2, double radius , int &numero , double &t_r , double &u_r , double &v_r) ;
 
  double c_c_c(double x , double y , double phi , double radius , double &t , double &u ,double &v);
  double c_cc(double x , double y , double phi , double radius , double &t , double &u , double &v);
  double csca(double x , double y , double phi , double radius , double &t , double &u ,double& v);
  double cscb(double x ,  double y , double phi , double radius , double &t , double &u ,double &v);
  double ccu_cuc(double x , double y , double phi , double radius , double &t , double &u ,double& v);
  double c_cucu_c(double x , double y , double phi , double radius , double &t , double &u , double &v);
  double c_c2sca(double x , double y , double phi , double radius , double &t , double &u ,double &v);
  double c_c2scb(double x , double y , double phi , double radius , double &t , double &u , double &v);
  double c_c2sc2_c(double x , double y , double phi , double radius , double &t , double &u, double &v);
  double cc_c(double x , double y  , double phi , double radius , double &t , double &u , double &v);
  double csc2_ca(double x , double y , double phi , double radius , double &t , double &u , double &v);
  double csc2_cb(double x , double y , double phi , double radius , double &t , double &u ,double &v);

  /**
     @}
  */


        
};

/**
   @}
*/
#endif

