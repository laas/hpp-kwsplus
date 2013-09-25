/*
  Copyright CNRS-AIST 
  Authors: Florent Lamiraux
*/

#ifndef FLIC_MANAGER_H
#define FLIC_MANAGER_H

#include <hpp/kwsplus/direct-path/flic-direct-path.hh>

/**
   \brief class that contain some fonction to deal with flicDirectPath.
*/

class CflicManager
{
public:
  /** 
      \brief Canonical curve associated to a configuration
      \param fconf : a configuration represented by position orientation and curvature.
      \param s : arc-length abscissa along the canonical curve.
      \param deriv_order: desired order of derivatives.
      \retval Tab_gamma : array of derivatives of the flat output up to desired order: [x,y,x',y',x'',y'',...].
  */
  static void flatGamma(const CflicDirectPath::TflatConfig *fconf, double s, double deriv_order, double *Tab_gamma);
	
  /** 
      \brief Interpolation function alpha and derivatives up to desired order.
      \param u: parameter of function alpha.
      \param deriv_order: desired order of derivatives returned for alpha
      \retval Tab_alpha : array of derivatives of alpha at u up to desired order.

      alpha is a real valued function that needs to satisfy the following equalities: alpha(0)=0, alpha(1)=1, alpha'(0)=0, alpha'(1)=0, , alpha''(0)=0, alpha''(1)=0. In our case, alpha(u) = 6 u^5 -15 u^4 + 10 u^3.
  */
  static void flatAlpha(double u, int deriv_order, double *Tab_alpha) ;
	
		
  /**
     \brief Combination of canonical curves. \image html equation-combinaison-convexe.png ""
     \param fconf_initPt: initial configuration q1.
     \param finalFlatConfPt: final configuration q2
     \param u: parameter along the resulting curve (between 0 and 1)
     \param v2: parameter scaling of canonical curves
     \param deriv_order: desired order of derivatives
     \retval Tab_gamma: array of combination curve in the plane and derivatives up to desired order: [x,y,x',y',x'',y'',...].
  */
  static void flatCombination(const CflicDirectPath::TflatConfig *fconf_initPt, 
			      const CflicDirectPath::TflatConfig *finalFlatConfPt, double u, 
			      double v2, int deriv_order, double *Tab_gamma) ;

  /**
     \brief Conversion from an array of flat output derivatives to TflatConfig.
     \param tabDeriv Array of flat output derivatives:  \f$ \left(\gamma_x,\gamma_y,\gamma'_x,\gamma'_y,\gamma''_x,\gamma''_y\right) \f$ .
     \retval fconf Flat configuration:  \f$ \left(x,y,\tau,\kappa\right) \f$ .
     \param forward forward or backward.
  */
  static void flatConvDerivFlatconfig(double *tabDeriv, CflicDirectPath::TflatConfig *fconf, bool forward);
  
  /**
     \brief compute the upper and lower bounds of a function on an interval given an upper bound of absolute value of the derivative of the function, and the values of the functions at the bounds of the interval.
     \param boundInterval specify the bounds of the interval and store the resulting lower and upper bound of the function.
     \param valueUmin value of the function at the beginning of the interval.
     \param valueUmax value of the function at the end of the interval.
     \param derivativeUpperBound upper bound of the derivative of the function over the interval.
  */
  static ktStatus computeFunctionBoundsFromDerivativeUpperBound(TflicBoundInterval& boundInterval,
								double valueUmin, double valueUmax,
								double derivativeUpperBound);
  static double angleLimit(double angle);
};

#endif
