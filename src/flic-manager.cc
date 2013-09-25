/*
  Copyright CNRS-AIST 
  Authors: Florent Lamiraux
*/

#include "flic-manager.hh"

/// \brief Maximal order of derivation in combination function
#define MAX_DERIV_ORDER_IN_COMBINATION   5


// =========================================================================================

void CflicManager::flatGamma(const CflicDirectPath::TflatConfig *fconf, double s, double deriv_order, double *Tab_gamma)
{

  double Sin, Cos, teta, kappa;

  teta = fconf->tau;
  kappa = fconf->kappa;


  /* le cas d'un cercle */
  if(fabs(kappa) > NEAR_ZERO)
    {
      Sin = sin(teta + kappa*s);
      Cos = cos(teta + kappa*s);
      /* gamma_x */
      Tab_gamma[0] = fconf->xp + (Sin - sin(teta))/kappa;
      /* gamma_y */
      Tab_gamma[1] = fconf->yp - (Cos - cos(teta))/kappa;
      /* gamma_prime_x */
      Tab_gamma[2] = Cos;
      /* gamma_prime_y */
      Tab_gamma[3] = Sin;
    }
  /* approximation par une droite */
  else
    {
      Sin = sin(teta);
      Cos = cos(teta);
      /* gamma_x */
      Tab_gamma[0] = fconf->xp + s*Cos - .5*kappa*s*s*Sin ;
      /* gamma_y */
      Tab_gamma[1] = fconf->yp + s*Sin + .5*kappa*s*s*Cos;
      /* gamma_prime_x */
      Tab_gamma[2] = Cos - kappa*s*Sin;
      /* gamma_prime_y */
      Tab_gamma[3] = Sin + kappa*s*Cos;
    }
  /* gamma_sec_x */
  Tab_gamma[4] = -kappa*Sin;
  /* gamma_sec_y */
  Tab_gamma[5] = kappa*Cos;

  if (deriv_order < 3)
    return;

  /* gamma_tierce_x */
  Tab_gamma[6] = -kappa*kappa*Cos;
  /* gamma_tierce_y */
  Tab_gamma[7] = -kappa*kappa*Sin;

  if (deriv_order < 4)
    return;

  /* gamma_quatre_x */
  Tab_gamma[8] = kappa*kappa*kappa*Sin;
  /* gamma_quatre_y */
  Tab_gamma[9] = -kappa*kappa*kappa*Cos;

  if (deriv_order < 5)
    return;

  /* gamma_cinq_x */
  Tab_gamma[10] = kappa*kappa*kappa*kappa*Cos;
  /* gamma_cinq_y */
  Tab_gamma[11] = kappa*kappa*kappa*kappa*Sin;
}

//=========================================================================================

void CflicManager::flatAlpha(double u, int deriv_order, double *Tab_alpha)
{

  /* a et b sont respectivement les derivees troisiemes de alpha en 0 et 1*/
  double u2, u3, u4, u5;
  static double   a3 = 10;
  static double a4 = -15;
  static double a5 = 6;

  u2 = u*u;
  u3 = u*u2;
  u4 = u*u3;
  u5 = u*u4;


  /* alpha(u) */
  Tab_alpha[0] = a3*u3 + a4*u4 + a5*u5;
  /* alpha_prime(u) */
  Tab_alpha[1] = 3.0*a3*u2 + 4.0*a4*u3 + 5.0*a5*u4;
  /* alpha_sec(u) */
  Tab_alpha[2] = 6.0*a3*u + 12.0*a4*u2 + 20.0*a5*u3;

  if (deriv_order < 3)
    return;

  /* alpha_tierce(u) */
  Tab_alpha[3] = 6.0*a3 + 24.0*a4*u + 60.0*a5*u2;
  if (deriv_order < 4)
    return;

  /* alpha_quatre(u) */
  Tab_alpha[4] = 24.0*a4 + 120.0*a5*u;
  if (deriv_order < 5)
    return;

  /* alpha_cinq(u) */
  Tab_alpha[5] = 120.0*a5;
}

//=========================================================================================

void CflicManager::flatCombination(const CflicDirectPath::TflatConfig *fconf_initPt,
				   const CflicDirectPath::TflatConfig *finalFlatConfPt, double u,
				   double v2, int deriv_order, double *Tab_gamma)
{


  double tab_gamma_1[2*(MAX_DERIV_ORDER_IN_COMBINATION+1)];
  double tab_gamma_2[2*(MAX_DERIV_ORDER_IN_COMBINATION+1)];
  double tab_alpha[MAX_DERIV_ORDER_IN_COMBINATION+1];
  double gamma2_1_x, gamma2_1_y;
  double gamma2_1_x_prime, gamma2_1_y_prime;
  double gamma2_1_x_sec, gamma2_1_y_sec;
  double gamma2_1_x_tierce, gamma2_1_y_tierce;


  flatGamma(fconf_initPt, u*v2, deriv_order, tab_gamma_1);
  flatGamma(finalFlatConfPt, (u-1.0)*v2, deriv_order, tab_gamma_2);
  flatAlpha(u, deriv_order, tab_alpha);


  gamma2_1_x = tab_gamma_2[0] - tab_gamma_1[0];
  gamma2_1_y = tab_gamma_2[1] - tab_gamma_1[1];

  /* x */
  Tab_gamma[0] = tab_alpha[0]*gamma2_1_x + tab_gamma_1[0];
  /* y */
  Tab_gamma[1] = tab_alpha[0]*gamma2_1_y + tab_gamma_1[1];

  gamma2_1_x_prime = tab_gamma_2[2] - tab_gamma_1[2];
  gamma2_1_y_prime = tab_gamma_2[3] - tab_gamma_1[3];
  /* x_prime */
  Tab_gamma[2] = tab_alpha[1]*gamma2_1_x +
    tab_alpha[0]*v2*gamma2_1_x_prime + v2*tab_gamma_1[2];
  /* y_prime */
  Tab_gamma[3] = tab_alpha[1]*gamma2_1_y +
    tab_alpha[0]*v2*gamma2_1_y_prime + v2*tab_gamma_1[3];

  gamma2_1_x_sec = tab_gamma_2[4] - tab_gamma_1[4];
  gamma2_1_y_sec = tab_gamma_2[5] - tab_gamma_1[5];
  /* x_sec */
  Tab_gamma[4] = tab_alpha[2]*gamma2_1_x +
    2.0*tab_alpha[1]*v2*gamma2_1_x_prime +
    tab_alpha[0]*v2*v2*gamma2_1_x_sec +
    v2*v2*tab_gamma_1[4];
  /* y_sec */
  Tab_gamma[5] = tab_alpha[2]*gamma2_1_y +
    2.0*tab_alpha[1]*v2*gamma2_1_y_prime +
    tab_alpha[0]*v2*v2* gamma2_1_y_sec +
    v2*v2*tab_gamma_1[5];



  if (deriv_order < 3)
    return;

  gamma2_1_x_tierce = tab_gamma_2[6] - tab_gamma_1[6];
  gamma2_1_y_tierce = tab_gamma_2[7] - tab_gamma_1[7];
  /* x_tierce */
  Tab_gamma[6] = tab_alpha[3]*gamma2_1_x +
    3.0*tab_alpha[2]*v2*gamma2_1_x_prime +
    3.0*tab_alpha[1]*v2*v2*gamma2_1_x_sec +
    tab_alpha[0]*v2*v2*v2* gamma2_1_x_tierce +
    v2*v2*v2*tab_gamma_1[6];
  /* y_tierce */
  Tab_gamma[7] = tab_alpha[3]*gamma2_1_y +
    3.0*tab_alpha[2]*v2*gamma2_1_y_prime +
    3.0*tab_alpha[1]*v2*v2* gamma2_1_y_sec +
    tab_alpha[0]*v2*v2*v2*gamma2_1_y_tierce +
    v2*v2*v2*tab_gamma_1[7];

  if (deriv_order < 4)
    return;

  /* x_quatre */
  Tab_gamma[8] = tab_alpha[4]*gamma2_1_x +
    4.0*tab_alpha[3]*v2*gamma2_1_x_prime +
    6.0*tab_alpha[2]*v2*v2*gamma2_1_x_sec +
    4.0*tab_alpha[1]*v2*v2*v2*gamma2_1_x_tierce +
    tab_alpha[0]*v2*v2*v2*v2*(tab_gamma_2[8] - tab_gamma_1[8]) +
    v2*v2*v2*v2*tab_gamma_1[8];
  /* y_quatre */
  Tab_gamma[9] = tab_alpha[4]*gamma2_1_y +
    4.0*tab_alpha[3]*v2*gamma2_1_y_prime +
    6.0*tab_alpha[2]*v2*v2* gamma2_1_y_sec +
    4.0*tab_alpha[1]*v2*v2*v2*gamma2_1_y_tierce +
    tab_alpha[0]*v2*v2*v2*v2*(tab_gamma_2[9] - tab_gamma_1[9]) +
    v2*v2*v2*v2*tab_gamma_1[9];

  if (deriv_order < 5)
    return;

  /* x_cinq */
  Tab_gamma[10] = tab_alpha[5]*gamma2_1_x +
    5.0*tab_alpha[4]*v2*gamma2_1_x_prime +
    10.0*tab_alpha[3]*v2*v2*gamma2_1_x_sec +
    10.0*tab_alpha[2]*v2*v2*v2*gamma2_1_x_tierce +
    5.0*tab_alpha[1]*v2*v2*v2*v2*(tab_gamma_2[8] - tab_gamma_1[8]) +
    tab_alpha[0]*v2*v2*v2*v2*v2*(tab_gamma_2[10] - tab_gamma_1[10])+
    v2*v2*v2*v2*v2*tab_gamma_1[10];
  /* y_cinq */
  Tab_gamma[11] = tab_alpha[5]*gamma2_1_y +
    5.0*tab_alpha[4]*v2*gamma2_1_y_prime +
    10.0*tab_alpha[3]*v2*v2*gamma2_1_y_sec +
    10.0*tab_alpha[2]*v2*v2*v2*gamma2_1_y_tierce +
    5.0*tab_alpha[1]*v2*v2*v2*v2*(tab_gamma_2[9] - tab_gamma_1[9]) +
    tab_alpha[0]*v2*v2*v2*v2*v2*(tab_gamma_2[11] - tab_gamma_1[11])+
    v2*v2*v2*v2*v2*tab_gamma_1[11];
}

// ==========================================================================================


double CflicManager::angleLimit(double angle)
{
  while (angle < -M_PI)
    {
      angle += 2*M_PI;
    }
  while (angle > M_PI)
    {
      angle -= 2*M_PI;
    }
  return angle;
}

// ==========================================================================================


void CflicManager::flatConvDerivFlatconfig(double *tabDeriv, CflicDirectPath::TflatConfig *fconf, bool forward)
{
  double curve_prime_x, curve_prime_y;

  curve_prime_x = tabDeriv[2];
  curve_prime_y = tabDeriv[3];


  fconf->xp = tabDeriv[0];
  fconf->yp = tabDeriv[1];
  if(!curve_prime_x)
    {
      fconf->tau = M_PI/2.0;
      if(curve_prime_y < 0.0) fconf->tau *= -1.0;
    }
  else
    {
      fconf->tau = atan(curve_prime_y/curve_prime_x);
      if(curve_prime_x < 0.0)
	{
	  fconf->tau += M_PI;
	}
    }
  if(!forward)
    {
      fconf->tau = angleLimit(fconf->tau + M_PI);
      fconf->kappa = -(curve_prime_x*tabDeriv[5] - curve_prime_y*tabDeriv[4])/
	pow(curve_prime_x*curve_prime_x + curve_prime_y*curve_prime_y, 1.5);
    }
  else
    {
      fconf->tau = angleLimit(fconf->tau);
      fconf->kappa = (curve_prime_x*tabDeriv[5] - curve_prime_y*tabDeriv[4])/
	pow(curve_prime_x*curve_prime_x + curve_prime_y*curve_prime_y, 1.5);
    }
}

// ==========================================================================================


ktStatus CflicManager::computeFunctionBoundsFromDerivativeUpperBound(TflicBoundInterval& boundInterval,
								    double valueUmin, double valueUmax,
								    double derivativeUpperBound)
{

  double u1, u2 ;
  double meanValue ;
  double halfDiffBound ;

  u1 = boundInterval.uMin ;
  u2 = boundInterval.uMax ;

  if (u1 > u2)
    {
      return KD_ERROR ;
    }

  meanValue  = 0.5 * (valueUmin + valueUmax) ;
  halfDiffBound = 0.5 * derivativeUpperBound * (u2 -u1) ;

  boundInterval.valueMin = meanValue - halfDiffBound ;
  boundInterval.valueMax = meanValue + halfDiffBound ;

  return KD_OK ;


}



