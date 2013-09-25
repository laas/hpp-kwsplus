/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Florent Lamiraux (LAAS-CNRS)
           and Mathieu Poirier (LAAS-CNRS)

*/

#ifndef __FLATINTERCARTDP_H
#define __FLATINTERCARTDP_H

/*************************************
INCLUDE
**************************************/

#include <iostream>
#include <list>
#include "KineoWorks2/kwsSteeringMethod.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDefine.h"

#include <hpp/kwsplus/direct-path/direct-path.hh>

KIT_PREDEF_CLASS( CflicDirectPath );
KIT_PREDEF_CLASS( CflicBoundManagerDefParam );
KIT_PREDEF_CLASS( CflicArcLengthManager );
KIT_PREDEF_CLASS( CflicBoundManagerArcLengthParam );

/** 

\addtogroup flic 
@{
\section intro Introduction

 This part implements FLat ORiENTed INtErpolation direct path and local method for a cart-like mobile robot described in \ref paper-florentine "Lamiraux Laumond 2001". The method uses the flatness property of the cart mobile robot and builds paths through planar curves with given position, orientation and curvature at both ends. According to the flatness property, any admissible trajectory of the cart can be represented by the curve followed by the center of the cart. The orientation of the cart is the direction of the tangent vector to the curve.

\section flat-section FLat ORiENTed INtErpolation for cart-like mobile robot

\subsection def-convex-combination Definition of the curves

The FLat ORiENTed INtErpolation between two configurations is defined as follows:
 \f{eqnarray*}
 {\bf q}_1 &=& (x_1,y_1,\tau_1,\kappa_1) \\
 {\bf q}_2 &=& (x_2,y_2,\tau_2,\kappa_2) 
 \f}
 where  \f$  \tau_{1,2} \f$  and  \f$  \kappa_{1,2} \f$  are respectively the orientation of the tangent vector and the desired curvature relative to configurations  \f$ {\bf q}_1  \f$  and  \f$ { \bf q}_2  \f$ . 

 To any configuration  \f$ {\bf q}=(x,y,\tau,\kappa) \f$ , we associate a circle (or straight line if the curvature is zero) denoted as  \f$ \Gamma({\bf q},s) \f$  starting from  \f$ {\bf q} \f$  ( \f$ \Gamma({\bf q},0)={\bf q} \f$ ) and parameterized by arc-length.

 We define  \f$  \bar{\bf q}_1  \f$  the projection of  \f$  \bar{\bf q}_1  \f$  on  \f$  \Gamma({\bf q}_1,.) \f$  and  \f$  v  \f$  the abscissa of this projection:
 \f{eqnarray*}
 \bar{\bf q}_1 &=& \Gamma({\bf q}_1,v)
 \f}
 We defines then the two following curves:
 \f{eqnarray*}
 \gamma_1(u) &=& \Gamma({\bf q}_1, v u) \\
 \gamma_2(u) &=& \Gamma({\bf q}_2, v (u-1))
 \f}

 If  \f$  \alpha  \f$  is a polynomial function satisfying:
 \f{eqnarray*}
 \alpha(0)=0 &&  \alpha(1)=1 \\
 {\alpha'}(0)=0 &&  \alpha'(1)=0 \\
 \alpha''(0)=0 &&  \alpha''(1)=0 
 \f}
where \f$\alpha'\f$ represents the derivative of \f$\alpha\f$,
 then the following flat interpolation
 \f[
 \gamma(u) = (1-\alpha(u))\gamma_1(u) + \alpha(u)\gamma_2(u)
 \f]
 represents a path between  \f$ {\bf q}_1 \f$  and  \f$ {\bf q}_2  \f$ .

 \image html combinaison-convexe.png "FLat ORiENTed INtErpolation: Each configuration is represented by a position in the plane, an orientation and a curvature (set to 0 for cart mobile robots unlike on the figure). To each configuration is associated a canonical curve (i.e. the circle or straight line passing through the point with corresponding orientation and curvature. Two curves drawn on these canonical curves are defined in such a way that the first one starts at q1 while the second one ends at q2. Thus by interpolating these two curves by a function going from 0 to 1 with first and second derivatives equal to 0 at both ends of the definition interval, we get a curve in the plane corresponding to an admissible trajectory going from q1 to q2 through the flatness property."

\section implementation Implementation

\subsection upper-bounds Upper-bounds of derivatives

When defining a derived class direct path, KineoWorks requires that function 
CflicDirectPath::maxAbsoluteDerivative be defined. This function computes an upper bound 
of the derivative of each dof over an interval given as input. For general curves, this
computation is intricate. Class CflicBoundManagerDefParam performs these computations
by building at the creation of the direct path data-structure that store lower and upper 
bounds of  \f$ \|\gamma'\| \f$  and  \f$ \|\gamma''\| \f$  over sub-intervals. See \ref rapport-mathieu "Poirier 2006" and Figure below for details.

\image html boundsD2gammaOverDu2.png "Lower and upper bounds of second derivative of gamma wrt u over sub-intervals."

\image html boundsDgammaOverDu.png "Lower and upper bounds of first derivative of gamma wrt u  over sub-intervals."


\subsection curvilinear-param Curvilinear parameterization

The planar curve  \f$  \gamma  \f$  representing a path for a cart-like mobile robot is not parameterized by curvilinear 
abscissa. For some applications, it is desirable to parameterize direct paths using  \f$  \gamma  \f$  arc-length. 
This is done in a second step by Class CflicArcLengthManager.
Mappings from default parameter u to arc-length parameter s and form s to u 
are approximated by piecewise polynomial functions of degree 3 (class CflicPiecewisePolynomial3). The figure below shows mappings from u to s and from s to u.

\image html UtoSandUtoS.png "Mappings from default parameter to arc-length parameter and from arc-length parameter to default."

\subsection upper-bounds-arc-length Upper-bounds of derivatives wrt arc-length

Function CflicDirectPath::maxAbsoluteDerivative needs to compute upper values of the dof derivatives with respect to arc-length over intervals. We thus need to compute these bounds as above. We use the same technique.

\image html boundsD2gammaOverDs2.png "Lower and upper bounds of second derivative of gamma wrt s over sub-intervals."

\image html boundsDgammaOverDs.png "Lower and upper bounds of first derivative of gamma wrt s over sub-intervals."

 \section References
 \anchor paper-florentine
 F. Lamiraux and J.-P. Laumond, "Smooth motion planning for car-like vehicles", IEEE Transaction on Robotics and Automation, vol 17, No 4, August 2001.\sa http://www.laas.fr/~florent/publication.html


 \anchor rapport-mathieu 
 M. Poirier, <a href="./papers/rapport-mathieu.pdf">"Planification de la locomotion humanoide"</a>, Rapport Master EEAS, juin 2006

 */

 

/*************************************
CONSTANTE
**************************************/

/// \brief define to compute integral
#define NEAR_ZERO 0.00000001

/// \brief the Epsilon between 2 scales
#define EPSILONINTER 0.01

/*************************************
STRUCTURES
**************************************/

/** 
    \brief Structure representing an upper and a lower bound of a function over an interval.
    
    \param uMin : Lower bound of interval
    \param uMax : Upper bound of interval
    \param valueMax : Upper bound of the function (derivative of  \f$ \gamma \f$ )
    \param valueMin : Lower bound of the function (derivative of  \f$ \gamma \f$ )
    \param fs : pointer on the next tree's cell
    \param ts : pointer on the tree's root of the inferior derivate tree
*/

typedef struct TflicBoundInterval { 
  double uMin ; 
  double uMax ; 
  double valueMax ; 
  double valueMin ; 
} TflicBoundInterval; 


// For test only
class CtestFlicDirectPath;

// ==============================================================================
//
//  CLASS CflicPolynomial3
//
// ==============================================================================


/**
   \brief Polynomial of degree 3 restricted to an interval  \f$ [u_1,u_2] \f$ . 
   The value of the polynomial at parameter u is given by the following formula:

   \f[
      P(u) = -\frac{(2(P(u_2)-P(u_1))-(u_2-u_1)(P'(u_1)+P'(u_2)))(u-u_1)^3}{(u_2 - u_1)^3} 
             -\frac{(-3(P(u_2)-P(u_1))(u_2-u_1)+P'(u_2)(u_2 - u_1)^2+2 P'(u_1)*(u_2 - u_1)^2)(u-u_1)^2}{(u_2 - u_1)^3}+ P'(u_1)(u-u_1)+P(u_1)
   \f]

*/

class CflicPolynomial3 {
  friend class CtestFlicDirectPath;
public:
  /**
     \brief Initialize polynomial with input values
     
     \param u1 lower bound of definition interval.
     \param u2 upper bound of definition interval.
     \param valueU1 value of polynomial for parameter u1.
     \param valueU2 value of polynomial for parameter u2.
     \param valueDerivU1 Value of polynomial derivative for parameter u2.
     \param valueDerivU2 of polynomial derivative for parameter u2.
  */
  CflicPolynomial3(double u1, double u2, double valueU1, double valueU2, 
		   double valueDerivU1, double valueDerivU2);
    
  CflicPolynomial3(): attU1(0),attU2(0),attValueU1(0),attValueU2(0),
    attValueDerivU1(0),attValueDerivU2(0),attFlagDerivativeBounds(false){};
    
  /** 
      \brief value of polynomial for input parameter.
  */
  double value(double u) const;
  /** 
      \brief value of the polynomial derivative for input parameter.
  */
  double valueDeriv(double u) const;
  /** 
      \brief value of the polynomial second derivative for input parameter.
  */
  double valueDeriv2(double u) const;
  /**
     \brief Return maximum absolute value of first derivative over definition interval.
  */
  double maxAbsDeriv1() const;
  /**
     \brief Return maximum absolute value of second derivative over definition interval.
  */
  double maxAbsDeriv2() const;
  /**
     \brief Return maximum absolute value of third derivative over definition interval.
  */
  double maxAbsDeriv3() const;
  /**
     \brief  Lower bound of definition interval.
  */
  double attU1;
  /**
     \brief  Upper bound of definition interval.
  */
  double attU2;
  /**
     \brief Value of polynomial for parameter u1.
  */
  double attValueU1;
  /**
     \brief Value of polynomial for parameter u2.
  */
  double attValueU2;
  /**
     \brief Value of polynomial derivative for parameter u1.
  */
  double attValueDerivU1;
  /**
     \brief Value of polynomial derivative for parameter u2.
  */
  double attValueDerivU2;

  /**
     \brief Compute bounds on the absolute value of first and second derivatives.
  */
  void computeDerivativeBounds();

private:
  /*
    \brief Compute the values of the polynomial coefficient wrt to values and derivatives in u1 and u2.
  */
  void computeCoefficients(double coef[4]) const;

  /*
    \brief Flag indicating whether bounds on derivatives have been computed.
    Set to false at construction.
  */
  bool attFlagDerivativeBounds;
  /**
     \brief Maximal absolute value of first derivative over interval of definition.
  */
  double attMaxAbsDeriv1;
  /**
     \brief Maximal absolute value of second derivative over interval of definition.
  */
  double attMaxAbsDeriv2;
  /**
     \brief Maximal absolute value of third derivative over interval of definition.
  */
  double attMaxAbsDeriv3;
};

/**
   \brief Piecewise degree 3 polynomial function.
   The function is defined over an interval  \f$ [u_{min}, u_{max}] \f$ , split into sub-intervals of constant length.
   The restriction of the function over each sub-interval is a polynomial of degree 3.
*/

class CflicPiecewisePolynomial3 {
  friend class CtestFlicDirectPath;
public:
  /**
     \name Constructors, destructor and initialization.
     @{
   */

  /**
     \brief Build a piecewise degree 3 polynomial with regular sampling step between inUmin and inUmax.

     Initialize the lower and upper bounds of each degree 3 polynomial in the vector.
  */
  CflicPiecewisePolynomial3(unsigned int inNbIntervals, double inUmin, double inUmax);
  /**
     \brief Set vector size to inNbIntervals.
  */
  CflicPiecewisePolynomial3(unsigned int inNbIntervals):
    attVectorPoly(inNbIntervals){};

  /**
     \brief Compute bounds on the absolute value of first and second derivatives for each sub-interval.
  */
  void computeDerivativeBounds();
  /**
     \brief Set the values of the lower and upper bounds of the definition interval of the function.

     attUmin is set to attU1 value of first Polynomial, attUmax is set to attU2 value of last Polynomial in vector.
  */
  ktStatus setDefIntervalBounds();

  /**
     @}
  */

  /**
     \name Getting definition interval, value and derivative.
     @{
  */

  /**
     \brief Value of the function for a given parameter.
  */
  double value(double u) const;
  /**
     \brief Value of the derivative for a given parameter.
  */
  double valueDeriv(double u) const;
  /**
     \brief return lower bound of interval.
  */
  double uMin() const {return attUmin;};
  /**
     \brief return upper bound of interval.
  */
  double uMax() const {return attUmax;};
  
  /**
     @}
  */
  /**
     \name Instantiating a degree 3 polynomial in the vector.
     @{
   */
  /**
     \brief Set attribute attU1 of polynomial of given rank
  */
  ktStatus u1(unsigned int rank, double inU1);

  /**
     \brief Set attribute attU1 of polynomial of given rank
  */
  ktStatus u2(unsigned int rank, double inU2);

  /**
     \brief Set attribute attValueU1 of polynomial of given rank
  */
  ktStatus valueU1(unsigned int rank, double inValueU1);

  /**
     \brief Set attribute attValueU2 of polynomial of given rank
  */
  ktStatus valueU2(unsigned int rank, double inValueU2);

  /**
     \brief Set attribute attValueDerivU1 of polynomial of given rank
  */
  ktStatus valueDerivU1(unsigned int rank, double inValueU1);

  /**
     \brief Set attribute attValueDerivU2 of polynomial of given rank
  */
  ktStatus valueDerivU2(unsigned int rank, double inValueU2);

  /**
     @}
  */

  /**
     \name Const iterators over vector of degree 3 polynomial.
     @{
   */
  /**
     \brief Iterator on first interval.
  */
  std::vector<CflicPolynomial3>::const_iterator begin() const;
  /**
     \brief Iterator past last interval.
  */
  std::vector<CflicPolynomial3>::const_iterator end() const;

  /**
     @}
  */

private:
  /**
     \brief Lower bound of the definition interval.
  */
  double attUmin;
  /**
     \brief Upper bound of the definition interval.
  */
  double attUmax;
  /**
   * \brief Vector of degree 3 polynomials.
   */
  std::vector<CflicPolynomial3> attVectorPoly;

 };



// ==============================================================================
//
//  CLASS CflicArcLengthManager
//
// ==============================================================================

/**
   \brief Manages curvilinear abscissa parameterization. 
*/

class CflicArcLengthManager {
  friend class CtestFlicDirectPath;
  friend class CflicDirectPath;
public: 
  /**
     \brief Store the number of samples in the array of parameter discretization.
  */
  CflicArcLengthManager(unsigned int nbSampleIntervals, CflicDirectPathShPtr inDirectPath);
  /** 
      \brief Defaut destructor
  */
  ~CflicArcLengthManager() { /* debug cout << "CflicArcLengthManager destructor" << endl; */ };

  /**
     \brief Return the value of the default parameter corresponding to given  \f$ \gamma \f$  arc length.
  */
  double defaultParam(double arcLength);

  /**
     \brief Return the value of the derivative of the default parameter corresponding to given  \f$ \gamma \f$  arc length.
  */
  double defaultParamDeriv(double arcLength);

  /**
     \brief Const iterator over degree 3 polynomials defining mapping from arc length to default parameters: first element.
  */
  std::vector<CflicPolynomial3>::const_iterator mappingArcLengthToDefIterBegin() const {return attArcLengthToDefault.begin();}; 
  /**
     \brief Const iterator over degree 3 polynomials defining mapping from arc length to default parameters: past last element.
  */
  std::vector<CflicPolynomial3>::const_iterator mappingArcLengthToDefIterEnd() const {return attArcLengthToDefault.end();};
private:
  /**
     \brief Build vector of polynomials representing the mapping between default and arc-length parameters.
  */
  void buildMappingDefaultToArcLength(CflicDirectPathShPtr inDirectPath);
  /**
     \brief Build vector of polynomials representing the mapping between arc-length and default parameters.
  */
  void buildMappingArcLengthToDefault();
  /**
     \brief Find default parameter corresponding to arc length parameter.
     
     Solve equation s=s(u) where mapping s() is defined by vector attDefaultToArcLength.

     \param arcLengthParam arc length parameter
     \return Default parameter.
  */
  double findDefaultParameter(double arcLengthParam);

  /**
     \brief Number of samples in the array of parameter discretization.

     The mapping from arc-length parameter to default parameter is stored in an array of 
     piece polynomials of degree 3 (cubic interpolation). This attribute stores the number of 
     sample intervals in the array.
  */
  unsigned int attNbSampleIntervals;
  /**
     \brief Mapping between default parameter and arc-length parameter.
     The mapping is stored in a vector of polynomials of degree 3 defined over sample intervals.
  */
  CflicPiecewisePolynomial3 attDefaultToArcLength;
  /**
     \brief Mapping between arc-length parameter and default parameter.
     The mapping is stored in a vector of polynomials of degree 3 defined over sample intervals.
  */
  CflicPiecewisePolynomial3 attArcLengthToDefault;
  /**
     \brief Arc Length of the direct path.
  */
  double attArcLength;
};


// ==============================================================================
//
//  CLASS CflicBoundManagerArcLengthParam
//
// ==============================================================================

/**
   \brief Manages lower and upper bounds of dof-derivatives wrt arc-length parameter
*/

class CflicBoundManagerArcLengthParam {
  friend class CtestFlicDirectPath;
  friend class CflicDirectPath;
public:

 /**
  * \brief Allocate a vector of TflicBoundInterval and store a pointer to the direct path.
  */
  CflicBoundManagerArcLengthParam(unsigned int nbSampleIntervals, double inArcLength,
                                  CflicDirectPathShPtr):
    attNbSampleIntervals(nbSampleIntervals), attArcLength(inArcLength), 
    boundVectorDeriv1(nbSampleIntervals), boundVectorDeriv2(nbSampleIntervals) {};

  ~CflicBoundManagerArcLengthParam() { /* debug cout << "CflicBoundManagerArcLengthParam destructor" << endl; */};
  /**
   * \brief Compute the upper and lower bounds of  \f$ \frac{d\gamma}{ds} \f$  over an interval
   * \param sMin Lower bound of interval.
   * \param sMax Upper bound of interval.
   * \retval valueMin Lower bound of  \f$ \frac{d\gamma}{ds} \f$ .
   * \retval valueMax Upper bound of  \f$ \frac{d\gamma}{ds} \f$ .
   */    
  void getBoundsNormDgammaOverDs(double sMin, double sMax, 
                                 double& valueMin, double& valueMax);
  /**
   * \brief Compute the upper and lower bounds of  \f$ \frac{d^2\gamma}{ds^2} \f$  over an interval
   * \param sMin Lower bound of interval.
   * \param sMax Upper bound of interval.
   * \retval valueMin Lower bound of  \f$ \frac{d^2\gamma}{ds^2} \f$ .
   * \retval valueMax Upper bound of  \f$ \frac{d^2\gamma}{ds^2} \f$ .
   */    
  void getBoundsNormD2gammaOverDs2(double sMin, double sMax, 
                                   double& valueMin, double& valueMax);
  /** 
   * \brief Return an upper bound of  \f$ \frac{d^3\gamma}{ds^3} \f$  over definition interval.
   */
   double maxD3gammaOverDs3() {return attMaxD3gammaOverDs3;};
private:
  /**
   * \brief Build vector of first derivative bounds of  \f$ \gamma \f$  w.r.t. arc-length.
   * The following formula is used to compute an upper bound of the second derivative:
   * \f{eqnarray*}
   * \frac{d^2\gamma}{ds^2}(s) &=& \gamma''(u(s)) \left(\frac{du}{ds}(s)\right)^2 + \gamma'(u(s)) \frac{d^2 u}{ds^2}\\
     \max_{s\in [s_{min},s_{max}]} \|\frac{d^2\gamma}{ds^2}(s)\| &=& \max_{u\in [u(s_{min}),u(s_{max})]}\|\gamma''(u)\|
     \max_{s\in [s_{min},s_{max}]} \left|\frac{du}{ds}(s)\right|^2 
     + \max_{u\in [u(s_{min}),u(s_{max})]}\|\gamma'(u)\| \max_{[s_{min},s_{max}]} \left|\frac{d^2u}{ds^2}(s)\right|
   * \f}
   * 
   */
  ktStatus buildBoundVectors(CflicDirectPathShPtr inDirectPath);
  /**
   * \brief Computes the minimal and maximal values of a function between two values of arc-length param.
   * \param boundIntVector Vector of lower and upper values of the function over sample intervals.
   * \param sMin Lower bound of interval over which bound is computer.
   * \param sMax Upper bound of interval over which bound is computer.
   * \retval valueMin Lower bound over interval.
   * \retval valueMax Upper bound over interval.
   */
  void getBoundsOnInterval(const std::vector<TflicBoundInterval>& boundIntVector, 
                           double sMin, double sMax, 
                           double& valueMin, double& valueMax);
  /**
   * \brief Number of sample intervals in the mapping from default to arc-length parameters.
   */
  unsigned int attNbSampleIntervals;
  /** 
   * \brief Arc length of the direct path.
   */
  double attArcLength;
  /** 
   * \brief Upper bound of  \f$ \frac{d^3\gamma}{ds^3} \f$  over definition interval.
   */
   double attMaxD3gammaOverDs3;
  /**
     \brief Vector of upper and lower bounds of  \f$ \|\frac{d\gamma}{ds}\| \f$  over sub-intervals of [0,S].
     s is the arc-length parameter, S is the total arc-lentgh of the direct path.
  */
  std::vector<TflicBoundInterval> boundVectorDeriv1;
   /**
     \brief Vector of upper and lower bounds of  \f$ \|\frac{d^2\gamma}{ds^2}\| \f$  over sub-intervals of [0,S].
     s is the arc-length parameter, S is the total arc-lentgh of the direct path.
  */
  std::vector<TflicBoundInterval> boundVectorDeriv2;
  
  
};


// ==============================================================================
//
//  CLASS CflicDirectPath
//
// ==============================================================================



/**
   \brief Direct path for flat interpolation for Cart.

   Direct path of this class have the following properties:
   \li they are parameterized by arc-length of the curve \f$\gamma \f$ followed by the flat output,
   \li the maximal curvature along the path is checked but not strictly enforced.
   
*/
class CflicDirectPath : public CkwsPlusDirectPath {
  friend class CtestFlicDirectPath;
  friend class CflicBoundManagerDefParam;
  friend class CflicArcLengthManager;
  friend class CflicBoundManagerArcLengthParam;
  friend class CflicDistance;

 public :
  /**
     \brief Indices of the different dofs implyied in flicDirectPath
  */
  enum EDofIndex {
    CURV_COORD = 0,
    X_COORD,
    Y_COORD,
    Z_COORD,
    RX_COORD,
    RY_COORD,
    RZ_COORD
  };

  /** 
      \brief Configuration structure for a flat interpolation.
      
      \param xp: abscissa of the flat output.
      \param yp: ordinate of the flat output.
      \param tau: (radian) direction of the unit tangent vector.
      \param kappa: curvature.
  */
  
  typedef struct TflatConfig {
    double xp;
    double yp;
    double tau;
    double kappa;
  } TflatConfig;


  /**
     \brief Delete boundManager and arcLengthManager.
  */
  ~CflicDirectPath();
   
  /**
     \brief  Create a new instance of a flat Interpolation cart direct path.
     \param  i_start 	: the start configuration
     \param  i_end 	: the end configuration
     \param  i_steeringMethod 	: shared pointer to the instance of the steering method that created the path
    \param inOriented : boolean , direct Path oriented or not
     \return shared pointer to a newly created flicDirectPath
  */
  static CflicDirectPathShPtr create(const CkwsConfig &i_start,
				     const CkwsConfig &i_end,
				     const CkwsSteeringMethodShPtr &i_steeringMethod,
				     bool inOriented);
  
  /**
     \brief  Creates by copy a new instance of a flat Interpolation cart direct path.
     \param i_flatDirectPath : a shared pointer to flicDirectPath that already exist
     \return shared pointer to a newly created flicDirectPath
  */
  static CflicDirectPathShPtr createCopy (const CflicDirectPathConstShPtr &i_flatDirectPath) ; 


  /**
     \brief clone : Returns a shared pointer to a newly allocated copy of the path.
     Note:
     If you want to avoid downcasting the result back to the subclass 
     of the parameter, use the appropriate create() method of the subclass
     so as to create a cloned copy of the object while preserving its type.
     \return shared pointer to a newly allocated copy
  */
  virtual CkwsAbstractPathShPtr clone() const;	

  /**
     \brief Get parameter v2
  */
  double flatV2() const {return attFlatV2;};
	
 protected :

  /**
     \brief  Constructor (non-default)
     \param inStartCfg : a start configuration
     \param inEndCfg   : a end configuration
     \param inSteeringMethod : a steering Method associed
  */
  CflicDirectPath(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg, const CkwsSteeringMethodShPtr &inSteeringMethod);
  /**
     \brief  Constructor (non-default)
     \param inDirectPath : a CkwsdirectPath to copy
  */ 
  CflicDirectPath(const CflicDirectPath &inDirectPath);
  
  /**
     \brief  init : initialise the weak pointer
     \param i_weakPTR : CkwsInterCartDPWkPtr
     \return KD_OK or KD_ERROR
  */
  ktStatus init(const CflicDirectPathWkPtr &i_weakPTR) ;

  /**
     \brief   Returns the parameter range of the direct path at the time it was built.
     \return initial parameter range of the direct path
     \note Private length is equal to \f$ \gamma \f$ -arc-length.
  */
  virtual double computePrivateLength() const ; 
  	
  /**
     \brief   interpolates between m_start and m_end, which are the start configuration
     and the goal configuration the direct path was initially built with.
     \param s 	: private distance ( \f$ \gamma \f$ -arc-length parameter)
     this value ranges from zero to privateLength().
     \param outCfg  : configuration on direct path
  */
  virtual void interpolate(double s, CkwsConfig & outCfg) const ;

  /**
     \brief   Return an overestimate of the absolute value of the derivatives wrt arc-length param of direct path between two positions on the path.

     Between inFrom and inTo, the absolute value of the i-th dof doesn't vary more than outDerivative[i] * (inTo - inFrom)
     \param inFrom  lower bound of interval.
     \param inTo  upper bound of interval.
     \param	outVectorDeriv : On return a vector that contains as many elements as there are dofs in the direct path and that is defined as explained above

     \note The mapping from  \f$ u \f$  to  \f$ s \f$  (and thus from  \f$ s \f$  to  \f$ u \f$  and from  \f$ s \f$  to  \f$ \gamma \f$ ) is of class  \f$ C^1 \f$  only. Therefore, the second derivative  \f$ \frac{d^2\gamma}{ds^2} \f$  is discontinuous. This implies that function CflicDirectPath::maxAbsoluteDerivative returns a wrong value for curvature dof. As long as curvature is not involved in collision checking, this problem has no consequence.
 
  */
  virtual void maxAbsoluteDerivative(double inFrom, double inTo, 
				     std::vector< double > & outVectorDeriv) const ;

  /**
     \brief   interpolates between m_start and m_end, which are the start configuration
     and the goal configuration the direct path was initially built with.
     \param u 	: defaut param  (between 0 and 1)
     \param outCfg  : on return, the interpolated configuration	
  */
  void interpolateDefaultParam(double u, CkwsConfig & outCfg) const ;
  
  /**
     \brief   Return an overestimate of the absolute value of the derivatives wrt default param of direct path between two positions on the path.

     Between inFrom and inTo, the absolute value of the i-th dof doesn't vary more than outDerivative[i] * (inTo - inFrom)
     \param inFrom  lower bound of interval.
     \param inTo  upper bound of interval.
     \param outVectorDeriv : On return a vector that contains as many elements as there are dofs in the
     direct path and that is defined as explained above
  */
  void maxAbsoluteDerivDefaultParam(double inFrom, double inTo, 
				    std::vector< double > & outVectorDeriv) const ;


  /**
     \brief Compute the velocity before extraction and reversion

     \param inDistance Distance along the path
     \retval outVelocity Derivative of the path with respect to distance parameter.
     \return KD_OK | KD_ERROR

  */
  virtual ktStatus getVelocityAtDistanceAtConstruction(double inDistance, std::vector<double>& outVelocity) const;

 private:
  /**
     \brief Returns the derivative of arc-length wrt default parameter:  \f$ \|\gamma'(u)\| \f$ .
  */
  double normGammaDeriv1(double u);

  /**
     \brief First and second derivatives of  \f$ \|\gamma\| \f$  wrt default param.
     
     \param u default parameter in  \f$ [0,1] \f$ .
     \retval xGamma_1 x component of  \f$ \frac{d\gamma}{ds}(u) \f$ .
     \retval yGamma_1 y component of  \f$ \frac{d\gamma}{ds}(u) \f$ .
     \retval xGamma_2 x component of  \f$ \frac{d^2\gamma}{ds^2}(u) \f$ .
     \retval yGamma_2 y component of  \f$ \frac{d^2\gamma}{ds^2}(u) \f$ .
  */
  ktStatus gammaDeriv1and2(double u, double& xGamma_1, double& yGamma_1,
                           double& xGamma_2, double& yGamma_2);

  /**
     \brief Compute the arc-length parameter v2 of  \f$ \bar{\bf q}_1 \f$  over curve  \f$ \Gamma({\bf q}_1,.)\f$.
     \param fconf1
     \param fconf2
  */
  double computeFlatV2(TflatConfig *fconf1, TflatConfig *fconf2);

  /** 
      \brief This object manages the computations required by function maxAbsoluteDerivDefaultParam.
      From an upper bound of the norm of third derivative of the flat output  \f$  \gamma(u) \f$ , lower and upper 
      bounds of the norm of the second and first derivative of  \f$\gamma \f$  are
      computed over sub intervals of [0,1]. Derivative are here with respect to default param.
  */
  CflicBoundManagerDefParamShPtr attBoundManagerDefParam;

  /** 
      \brief This object manages the computations required by function maxAbsoluteDerivative.
      The same computation are performed and the same data are stored except that the derivatives are 
      computed with respect to arc-length parameter instead of default parameter. 
  */
  CflicBoundManagerArcLengthParamShPtr attBoundManagerArcLengthParam;

  /**
     \brief this object manages the parameterization change between arc-length and default parameter.
  */
  CflicArcLengthManagerShPtr attArcLengthManager;

  // weak pointer to itself
  CflicDirectPathWkPtr m_weakPtr ; 
  /**
     \brief Curvilinear abscissa of projection of attFlatEndCfg on canonical curve relative to attFlatStartCfg.
  */
  double attFlatV2;
  /**
     \brief Flat configuration corresponding to startCfg
  */
  TflatConfig attFlatStartCfg;
  /**
     \brief Flat configuration corresponding to endCfg
  */
  TflatConfig attFlatEndCfg ;

  /**
     \brief Maximal curvature allowed along the path.

     If actual curvature goes beyond this bound, the direct path is rejected.
  */
  double attMaxCurvature;
};

// ==============================================================================
//
//  CLASS CflicBoundManagerDefParam
//
// ==============================================================================


/** 
    \brief This class manages the computations required by function CflicDirectPath::maxAbsoluteDerivative.
    From an upper bound of the norm of third derivative of the flat output  \f$ \gamma(u) \f$ , lower and upper bounds of the norm of the second and first derivative of  \f$ \gamma \f$  are
    computed over sub intervals of [0,privateLength]. This interval is recusively divided.
*/

class CflicBoundManagerDefParam {
  friend class CtestFlicDirectPath;
  friend class CflicDirectPath;
 public:
  /**
      \brief Store pointer to direct path owning object
   */
  CflicBoundManagerDefParam(CflicDirectPathShPtr inDirectPath, const CkwsConfig &inStartCfg, 
			    const CkwsConfig &inEndCfg);
  /**
     \brief Default destructor.
  */
  ~CflicBoundManagerDefParam();
  /**
     \brief build the list of upper and lower bounds of second and first derivative of  \f$ \gamma(u) \f$ 
  */
  ktStatus buildBoundLists();

  /**
   * \brief Get upper and lower bounds of  \f$ \|\gamma'\| \f$  over an interval.
   */
  void getBoundsNormGamma1(double uMin, double uMax, double& valueMin, double& valueMax); 
  /**
   * \brief Get upper and lower bounds of  \f$ \|\gamma''\| \f$  over an interval.
   */
  void getBoundsNormGamma2(double uMin, double uMax, double& valueMin, double& valueMax); 
  /**
   * \brief Get upper bound of  \f$ \|\gamma'''\| \f$ .
   */
  double getBoundsNormGamma3() {return M3;}; 
  /**
     \brief To access the boundListDeriv1 attribute
  */
  std::list<TflicBoundInterval>&  getBoundListDeriv1() { return boundListDeriv1 ; } ;
  /**
     \brief To access the boundListDeriv1 attribute
  */
  std::list<TflicBoundInterval>&  getBoundListDeriv2() { return boundListDeriv2 ; } ;

  /**
   * \brief returns the upper bound of  \f$ \|\gamma'''\| \f$ 
   */
  double getUpperBoundGamma3() { return M3; };


 private:

  /**
     \brief Find the min and max of the absolute overestimate in between bounds
     \param boundIntervalList : the list in which we are looking for
     \param uMin : lower bound of interval
     \param uMax : upper bound of interval
     \param minValue : min value  over interval
     \param maxValue : max value  over interval
  */
  void getBoundsOnInterval(std::list<TflicBoundInterval> boundIntervalList , double uMin, double uMax, double &minValue, double &maxValue); 

  /**
     \brief iterative algorithm to built the lists of absolute overestimate
     \param maxNbIntervals number of intervals in boundListDeriv1 if all were of the same size as the smallest one.
     \param boundsOnInterval1 iterator a list element representing lower and upper bounds of  \f$ \gamma' \f$  over an interval.
     \param boundsOnInterval2 iterator a list element representing lower and upper bounds of  \f$ \gamma'' \f$  over an interval.
     \param currentBoundIntervalCurv represents lower and upper bound on curvature over an interval.
     
     The first call of this recursive fonction takes as input two data-structures 
     representing a lower and upper bounds of the norm of  \f$ \gamma' \f$  
     and  \f$ \gamma'' \f$  respectively on interval [0,1]. 
     The function splits the interval into two sub-intervals of same length
     and recomputes the lower and upper bounds over each sub-interval if
     the difference between the upper and lower bounds of  \f$ \gamma' \f$ 
     if more than maxDistBetweenLowerAndUpperBound and does nothing otherwise.
  */

  ktStatus recursiveBuildBoundLists(unsigned int maxNbIntervals,
				    TflicBoundInterval  boundsOnInterval1,
				    TflicBoundInterval  boundsOnInterval2,
				    TflicBoundInterval& currentBoundIntervalCurv);

  /**
     \brief compute the upper bound of  \f$ \|\gamma'''\| \f$ 
  */
  double computeUpperBoundGamma3();

  /**
     \brief Compute  \f$ \|\gamma''(u)\| \f$ .
     \param u default parameter
  */
  double computeValueGamma2(double u);

  /**
     \brief Compute  \f$ \|\gamma'(u)\| \f$ 
     \param u default parameter
  */
  double computeValueGamma1(double u);
  /**
     \brief Initial configuration of direct path
  */
  const CkwsConfig &startCfg;
  /**
     \brief End configuration of direct path
  */
  const CkwsConfig &endCfg;
  /**
     \brief Flat configuration corresponding to startCfg
  */
  CflicDirectPath::TflatConfig attFlatStartCfg;
  /**
     \brief Flat configuration corresponding to endCfg
  */
  CflicDirectPath::TflatConfig attFlatEndCfg ;
  /**
     \brief Curvilinear abscissa of projection of attFlatEndCfg on canonical curve relative to attFlatStartCfg.
  */
  double attFlatV2;
 
  /**
     \brief Precision of upper and lower bounds threshold. 
     Whenever the difference between lower and upper bounds of the norm 
     of  \f$ \gamma' \f$  is less than this value for each 
     sub-interval, recursive interval spitting stops.
  */
  double maxDistBetweenLowerAndUpperBound;
  /**
     \brief Threshold for  \f$ \|\gamma'\| \f$  below which the direct path is discarded.
  */
  double minNormGamma1;
  /**
     \brief Maximum value of the norm of  \f$ \gamma''' \f$ .
  */
  double M3;
  
  /**
     \brief number of intervals in boundListDeriv1 if all were of the same size as the smallest one.
     This number defines the number of intervals in arc-length parameter sampling.
  */
  unsigned int attMaxNbIntervals;

  /**
     \brief list of upper and lower bounds of  \f$ \|\gamma'\| \f$  over sub-intervals of [0,1].

  */
  std::list<TflicBoundInterval> boundListDeriv1;
  /**
     \brief list of upper and lower bounds of  \f$ \|\gamma''\| \f$  over sub-intervals of [0,1].

  */
  std::list<TflicBoundInterval> boundListDeriv2;

  /**
     \brief maximal curvature of the associated direct path.
  */
  double attMaxCurvature;
};


/**
   @}
*/
#endif
