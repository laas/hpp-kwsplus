/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Developed by Florent Lamiraux
           and Mathieu Poirier (LAAS-CNRS)
 
*/


/*****************************************
INCLUDES
*******************************************/

#include "flicDirectPath.h"
#include "KineoWorks2/kwsDevice.h" // without this causes error when calling functions of kwsDevice
#include "kwsioInterface.h"
#include <math.h>
#include <fstream>


/** 
    \brief Canonical curve associated to a configuration
    \param fconf : a configuration represented by position orientation and curvature.
    \param s : arc-length abscissa along the canonical curve.
    \param deriv_order: desired order of derivatives.
    \retval Tab_gamma : array of derivatives of the flat output up to desired order: [x,y,x',y',x'',y'',...].
*/
static void flatGamma(CflicDirectPath::TflatConfig *fconf, double s, double deriv_order, double *Tab_gamma) ;
	
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
static void flatCombination(CflicDirectPath::TflatConfig *fconf_initPt, 
			    CflicDirectPath::TflatConfig *finalFlatConfPt, double u, 
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




CflicPolynomial3::CflicPolynomial3(double u1, double u2, double valueU1, double valueU2,
                                   double valueDerivU1, double valueDerivU2):
    attU1(u1),attU2(u2),attValueU1(valueU1),attValueU2(valueU2),
    attValueDerivU1(valueDerivU1),attValueDerivU2(valueDerivU2)
{
  computeDerivativeBounds();
}


//==============================================================================

double CflicPolynomial3::value(double u) const
{

  /* derivee de phi par rapport a la courbure aux extremites */
  double h = attU2-attU1;
  double h2 = h*h;
  double h3 = h2*h;

  double du = u-attU1;
  double du2 = du*du;
  double du3 = du*du2;

  double dy = attValueU2-attValueU1;

  double val = -(2*dy-h*(attValueDerivU1+attValueDerivU2))*du3/h3 - (-3*dy*h+attValueDerivU2*h2+2*attValueDerivU1*h2)*du2/h3+attValueDerivU1*du+attValueU1;

  return val;
}


//==============================================================================

double CflicPolynomial3::valueDeriv(double u) const
{
  double derivative = -3*(2*attValueU2-2*attValueU1-(attU2-attU1)*(attValueDerivU1+attValueDerivU2))*pow((u-attU1),2.0)*pow((attU2-attU1),-3.0)
                      -2*(-3*(attValueU2-attValueU1)*(attU2-attU1)+attValueDerivU2*pow((attU2-attU1),2.0)+2*attValueDerivU1*pow((attU2-attU1),2.0))*(u-attU1)*pow((attU2-attU1),-3.0)+attValueDerivU1;

  return derivative;
}

//==============================================================================

double CflicPolynomial3::valueDeriv2(double u) const
{
  double derivative = -6*(2*attValueU2-2*attValueU1-(attU2-attU1)*(attValueDerivU1+attValueDerivU2))*pow((attU2-attU1),-3.0)*(u-attU1)
                      -2*(-3*(attValueU2-attValueU1)*(attU2-attU1)+attValueDerivU2*pow((attU2-attU1),2.0)+2*attValueDerivU1*pow((attU2-attU1),2.0))*pow((attU2-attU1),-3.0);

  return derivative;
}

//==============================================================================

double CflicPolynomial3::maxAbsDeriv1() const
{
  if (!flagDerivativeBounds)
  {
    return -1;
  }
  return attMaxAbsDeriv1;
}

//==============================================================================

double CflicPolynomial3::maxAbsDeriv2() const
{
  if (!flagDerivativeBounds)
  {
    return -1;
  }
  return attMaxAbsDeriv2;
}

//==============================================================================

double CflicPolynomial3::maxAbsDeriv3() const
{
  if (!flagDerivativeBounds)
  {
    return -1;
  }
  return attMaxAbsDeriv3;
}

/*****************************************
 PRIVATE METHODS
*****************************************/


//==============================================================================

// Code generated by maple.

void CflicPolynomial3::computeCoefficients(double coef[4]) const
{
  // Coefficient of u^3.
  coef[3] = (-2 * attValueU1 + attU1 * attValueDerivU2 + attU1 * attValueDerivU1 - attU2 * attValueDerivU2 - attU2 * attValueDerivU1 + 2 * attValueU2) / ( pow((double) attU1, (double) 3) - 3 * attU2 * attU1 * attU1 -  pow((double) attU2, (double) 3) + 3 * attU1 * attU2 * attU2);
  // Coefficient of u^2.
  coef[2] = -(2 * attU1 * attU1 * attValueDerivU2 + attU1 * attU1 * attValueDerivU1 - 3 * attU1 * attValueU1 - attU1 * attU2 * attValueDerivU2 + 3 * attU1 * attValueU2 + attU1 * attU2 * attValueDerivU1 - 2 * attU2 * attU2 * attValueDerivU1 - 3 * attU2 * attValueU1 - attU2 * attU2 * attValueDerivU2 + 3 * attU2 * attValueU2) / ( pow((double) attU1, (double) 3) - 3 * attU2 * attU1 * attU1 -  pow((double) attU2, (double) 3) + 3 * attU1 * attU2 * attU2);
  // Coefficient of u.
  coef[1] = (pow(attU1, 0.3e1) * attValueDerivU2 + attU1 * attU1 * attU2 * attValueDerivU2 + 0.2e1 * attU1 * attU1 * attU2 * attValueDerivU1 - attU2 * attU2 * attU1 * attValueDerivU1 - 0.6e1 * attU1 * attU2 * attValueU1 - 0.2e1 * attU2 * attU2 * attU1 * attValueDerivU2 + 0.6e1 * attU1 * attU2 * attValueU2 - pow(attU2, 0.3e1) * attValueDerivU1) / (pow(attU1, 0.3e1) - 0.3e1 * attU2 * attU1 * attU1 - pow(attU2, 0.3e1) + 0.3e1 * attU1 * attU2 * attU2);
  // Constant coefficient.
  coef[0] = -(attValueU1 * pow(attU2, 0.3e1) - 0.3e1 * attValueU1 * attU1 * attU2 * attU2 + pow(attU1, 0.3e1) * attU2 * attValueDerivU2 - pow(attU1, 0.3e1) * attValueU2 + attU1 * attU1 * attU2 * attU2 * attValueDerivU1 - attU1 * attU1 * attU2 * attU2 * attValueDerivU2 + 0.3e1 * attU1 * attU1 * attU2 * attValueU2 - attU1 * attValueDerivU1 * pow(attU2, 0.3e1)) / (pow(attU1, 0.3e1) - 0.3e1 * attU2 * attU1 * attU1 - pow(attU2, 0.3e1) + 0.3e1 * attU1 * attU2 * attU2);

}


//==============================================================================

void CflicPolynomial3::computeDerivativeBounds()
{
  double coef[4];

  computeCoefficients(coef);

  // First derivative : P'(u) = 3 a3 u^2 + 2 a2 u + a1 = a u^2 + b u + c.
  double a = 3*coef[3];
  double b = 2*coef[2];
  double c = coef[1];

  if (fabs(a) < 1e-8)
  {
    if (fabs(b) < 1e-8)
    {
      // Polynomial is of degree 1, first derivative is constant.
      attMaxAbsDeriv1 = fabs(c);
      attMaxAbsDeriv2 = 0;
    }
    else
    {
      // Polynomial is of degree 2, first derivative is of degree 1.
      // Maximum of first derivative absolute value is reached at one of the bounds of the definition interval.
      attMaxAbsDeriv1 = max(fabs(b*attU1 + c), fabs(b*attU2 + c));
      // Second derivative is constant.
      attMaxAbsDeriv2 = fabs(b);
    }
  }
  else
  {
    // Polynomial is of degree 3, first derivative is of degree 2.
    // Interval bounds are potential parameter values for maximum absolute derivative.
    attMaxAbsDeriv1 = max(fabs(a*pow(attU1,2.0) + b*attU1 + c),
                          fabs(a*pow(attU2,2.0) + b*attU2 + c));
    // Potential maximal absolute value of first derivative can be reached at u = -b/(2a)
    // if this value is inside definition interval.
    double minusBover2a = -b/(2*a);

    if ((attU1 < minusBover2a ) && (minusBover2a < attU2))
    {
      // -b/(2a) is inside definition interval. Maximum absolute value
      // of first derivative might be reached for this parameter value.
      attMaxAbsDeriv1 = max(attMaxAbsDeriv1, fabs(a*pow(minusBover2a,2.0) + b*minusBover2a + c));
    }
    // Second derivative
    // P''(u) = 2a u + b
    // is of degree 1. Maximum absolute value reached at one bound of
    // definition interval.
    attMaxAbsDeriv2 = max (fabs(2*a*attU1+b), fabs(2*a*attU2+b));
  }
  // Third derivative is constant.
  attMaxAbsDeriv3 = 2*a;

  flagDerivativeBounds = true;
}




// ==============================================================================
//
//  METHOD OF CLASS CflicPiecewisePolynomial3
//
// ==============================================================================




/*****************************************
 PUBLIC METHODS
*****************************************/



CflicPiecewisePolynomial3::CflicPiecewisePolynomial3(unsigned int inNbIntervals, double inUmin, double inUmax):
    attUmin(inUmin),attUmax(inUmax),attNbIntervals(inNbIntervals),attVectorPoly(inNbIntervals)
{
  double sampleStep = (inUmax - inUmin)/inNbIntervals;
  // Fill definition interval for each polynomial.
  for (unsigned int iInterval=0; iInterval < inNbIntervals; iInterval++)
  {
    attVectorPoly[iInterval].attU1 = inUmin + iInterval*sampleStep;
    attVectorPoly[iInterval].attU2 = inUmin + (iInterval+1)*sampleStep;
  }
}


//==============================================================================

void CflicPiecewisePolynomial3::computeDerivativeBounds()
{
  std::vector<CflicPolynomial3>::iterator iter;
  for (iter=attVectorPoly.begin(); iter < attVectorPoly.end(); iter++)
  {
    iter->computeDerivativeBounds();
  }
}


//==============================================================================

ktStatus CflicPiecewisePolynomial3::setDefIntervalBounds()
{
  unsigned int vectorSize = attVectorPoly.size();
  if (vectorSize == 0)
  {
    cerr << "CflicPiecewisePolynomial3::setDefIntervalBounds: vector of polynomials is empty." << endl;
    return KD_ERROR;
  }
  attUmin = attVectorPoly[0].attU1;
  attUmax = attVectorPoly[vectorSize-1].attU2;

  return KD_OK;
}


//==============================================================================

double CflicPiecewisePolynomial3::value(double u) const
{
  if (attUmax <= attUmin)
  {
    cerr << "CflicPiecewisePolynomial3::value: attUmax <= attUmin. attUmin = "
    << attUmin << ", attUmax = " << attUmax << endl;
    return 0;
  }

  unsigned int polyId = (unsigned int)floor((u-attUmin)/(attUmax-attUmin)*attNbIntervals);
  if (u == attUmax)
  {
    polyId = attNbIntervals-1;
  }
  const CflicPolynomial3& poly3 = attVectorPoly[polyId];

  if ((u < poly3.attU1) || (u > poly3.attU2))
  {
    cerr << "CflicPiecewisePolynomial3::value: u out of range. u = " << u
    << ", poly3.attU1 = " << poly3.attU1
    << ", poly3.attU2 = " << poly3.attU2
    << ", polyId = " << polyId
    << ", attNbIntervals = " << attNbIntervals
    << endl;
  }
  double val = poly3.value(u);
  return val;
}

//==============================================================================

double CflicPiecewisePolynomial3::valueDeriv(double u) const
{
  if (attUmax <= attUmin)
  {
    cerr << "CflicPiecewisePolynomial3::valueDeriv: attUmax <= attUmin" << endl;
    return 0;
  }

  unsigned int polyId = (unsigned int)floor((u-attUmin)/(attUmax-attUmin)*attNbIntervals);
  if (u == attUmax)
  {
    polyId = attNbIntervals-1;
  }
  const CflicPolynomial3& poly3 = attVectorPoly[polyId];

  if ((u < poly3.attU1) || (u > poly3.attU2))
  {
    cerr << "CflicPiecewisePolynomial3::valueDeriv: u out of range"
    << ", poly3.attU1 = " << poly3.attU1
    << ", poly3.attU2 = " << poly3.attU2
    << ", polyId = " << polyId
    << ", attNbIntervals = " << attNbIntervals
    << endl;
  }
  double valueDeriv = poly3.valueDeriv(u);
  return valueDeriv;
}


//==============================================================================

ktStatus CflicPiecewisePolynomial3::u1(unsigned int rank, double inU1)
{
  if (rank >= attVectorPoly.size())
  {
    cerr << "CflicPiecewisePolynomial3::u1: rank bigger than vector size. rank = "
    << rank << "vector size = " << attVectorPoly.size() << endl;
    return KD_ERROR;
  }
  attVectorPoly[rank].attU1 = inU1;
  return KD_OK;
}

//==============================================================================

ktStatus CflicPiecewisePolynomial3::u2(unsigned int rank, double inU2)
{
  if (rank >= attVectorPoly.size())
  {
    cerr << "CflicPiecewisePolynomial3::u2: rank bigger than vector size. rank = "
    << rank << "vector size = " << attVectorPoly.size() << endl;
    return KD_ERROR;
  }
  attVectorPoly[rank].attU2 = inU2;
  return KD_OK;
}

//==============================================================================

ktStatus CflicPiecewisePolynomial3::valueU1(unsigned int rank, double inValueU1)
{
  if (rank >= attVectorPoly.size())
  {
    cerr << "CflicPiecewisePolynomial3::valueU1: rank bigger than vector size. rank = "
    << rank << "vector size = " << attVectorPoly.size() << endl;
    return KD_ERROR;
  }
  attVectorPoly[rank].attValueU1 = inValueU1;
  return KD_OK;
}

//==============================================================================

ktStatus CflicPiecewisePolynomial3::valueU2(unsigned int rank, double inValueU2)
{
  if (rank >= attVectorPoly.size())
  {
    cerr << "CflicPiecewisePolynomial3::valueU2: rank bigger than vector size. rank = "
    << rank << "vector size = " << attVectorPoly.size() << endl;
    return KD_ERROR;
  }
  attVectorPoly[rank].attValueU2 = inValueU2;
  return KD_OK;
}

//==============================================================================

ktStatus CflicPiecewisePolynomial3::valueDerivU1(unsigned int rank, double inValueDerivU1)
{
  if (rank >= attVectorPoly.size())
  {
    cerr << "CflicPiecewisePolynomial3::valueDerivU1: rank bigger than vector size. rank = "
    << rank << "vector size = " << attVectorPoly.size() << endl;
    return KD_ERROR;
  }
  attVectorPoly[rank].attValueDerivU1 = inValueDerivU1;
  return KD_OK;
}

//==============================================================================

ktStatus CflicPiecewisePolynomial3::valueDerivU2(unsigned int rank, double inValueDerivU2)
{
  if (rank >= attVectorPoly.size())
  {
    cerr << "CflicPiecewisePolynomial3::valueDerivU2: rank bigger than vector size. rank = "
    << rank << "vector size = " << attVectorPoly.size() << endl;
    return KD_ERROR;
  }
  attVectorPoly[rank].attValueDerivU2 = inValueDerivU2;
  return KD_OK;
}


//==============================================================================

std::vector<CflicPolynomial3>::const_iterator CflicPiecewisePolynomial3::begin() const
{
  std::vector<CflicPolynomial3>::const_iterator iter = attVectorPoly.begin();
  return iter;
}

//==============================================================================

std::vector<CflicPolynomial3>::const_iterator CflicPiecewisePolynomial3::end() const
{
  std::vector<CflicPolynomial3>::const_iterator iter = attVectorPoly.end();
  return iter;
}

// ==============================================================================
//
//  METHOD OF CLASS CflicBoundManagerDefParam
//
// ==============================================================================

void CflicBoundManagerDefParam::getBoundsNormGamma1(double uMin, double uMax, double& valueMin, double& valueMax)
{
  getBoundsOnInterval(boundListDeriv1, uMin, uMax, valueMin, valueMax);
}

//==============================================================================

void CflicBoundManagerDefParam::getBoundsNormGamma2(double uMin, double uMax, double& valueMin, double& valueMax)
{
  getBoundsOnInterval(boundListDeriv2, uMin, uMax, valueMin, valueMax);
}


//==============================================================================

void  CflicBoundManagerDefParam::getBoundsOnInterval(std::list<TflicBoundInterval> boundIntervalList, double uMin, double uMax, double &valueMin, double &valueMax)
{

  //-----debug
  //cout << " =====VERIF LISTE===== " << endl ;
  //for( std::list<TflicBoundInterval>::iterator itera = liste.begin(); itera !=liste.end() ; itera++ ) {
  //  cout << itera->uMin << "\t" << itera->uMax << "\t" << itera->valueMin << "\t" << itera->valueMax << endl ;
  //}
  //cout << " Get bounds Intervale " << endl ;
  //----------------------------------



  bool find = false ;

  std::list<TflicBoundInterval>::iterator iter = boundIntervalList.begin() ;

  while ((iter != boundIntervalList.end()) && !find )
  {
    if ((iter->uMin <= uMin) && (uMin < iter->uMax) ) find = true ;
    else iter++ ;
  }

  find = false ;

  valueMin = iter->valueMin ;
  valueMax = iter->valueMax ;


  while( (iter != boundIntervalList.end()) && !find )
  {
    if (iter->valueMin <= valueMin) valueMin = iter->valueMin ;
    if (iter->valueMax >= valueMax) valueMax = iter->valueMax ;
    if ((iter->uMin < uMax) && (uMax <= iter->uMax )) find = true ;

    iter ++ ;
  }


}

//==============================================================================

CflicBoundManagerDefParam::CflicBoundManagerDefParam(CflicDirectPathShPtr inDirectPath,
    const CkwsConfig &inStartCfg,
    const CkwsConfig &inEndCfg) :
    startCfg(inStartCfg),
    endCfg(inEndCfg),
    attMaxNbIntervals(0)
{
  //debug
  //cout << " CONSTRUCTEUR MANAGER  " << endl ;
  //------------------

  // Configuration convertion
  // assume that root joint is freeflyer
  flatStartCfg.kappa = inStartCfg.dofValue(CflicDirectPath::CURV_COORD) ; //Kappa (dof fictif)
  flatStartCfg.xp = inStartCfg.dofValue(CflicDirectPath::X_COORD) ;  // x
  flatStartCfg.yp = inStartCfg.dofValue(CflicDirectPath::Y_COORD) ;  // y
  flatStartCfg.tau = inStartCfg.dofValue(CflicDirectPath::RZ_COORD) ; // theta

  flatEndCfg.kappa = inEndCfg.dofValue(CflicDirectPath::CURV_COORD) ; //Kappa (dof fictif)
  flatEndCfg.xp = inEndCfg.dofValue(CflicDirectPath::X_COORD) ;  // x
  flatEndCfg.yp = inEndCfg.dofValue(CflicDirectPath::Y_COORD) ;  // y
  flatEndCfg.tau = inEndCfg.dofValue(CflicDirectPath::RZ_COORD) ; // theta

  attFlatV2 = inDirectPath->flatV2();

  M3 = computeUpperBoundGamma3();

  boundListDeriv1.clear();
  boundListDeriv2.clear();

  //debug
  //cout << "M3 : "  << M3  << " V2 : " << attFlatV2 << endl ;
  //--------------------------------
}

//==============================================================================

CflicBoundManagerDefParam::~CflicBoundManagerDefParam()
{
  // debug
  // cout << "Destructor of CflicBoundManagerDefParam" << endl;
  //----------------------
}

//==============================================================================

ktStatus CflicBoundManagerDefParam::recursiveBuildBoundLists(unsigned int maxNbIntervals,
    TflicBoundInterval currentBoundInterval1,
    TflicBoundInterval currentBoundInterval2)
{

  // find the intersection point
  double valueUmin =  computeValueGamma2(currentBoundInterval2.uMin) ;
  double valueUmax =  computeValueGamma2(currentBoundInterval2.uMax) ;
  computeFunctionBoundsFromDerivativeUpperBound(currentBoundInterval2, valueUmin,
      valueUmax, M3);

  valueUmin = computeValueGamma1(currentBoundInterval1.uMin) ;
  valueUmax = computeValueGamma1(currentBoundInterval1.uMax) ;
  computeFunctionBoundsFromDerivativeUpperBound(currentBoundInterval1, valueUmin,
      valueUmax,
      currentBoundInterval2.valueMax);

  // Test Threshold on gamma' norm
  if (currentBoundInterval1.valueMax <= minNormGamma1)
  {

    //cerr << " FAILURE -- currentBoundInterval1.valueMax <= minNormGamma1 " << currentBoundInterval1.valueMax <<" <= " << minNormGamma1 << endl ;

    return KD_ERROR;
  }
  // Test stop condition
  if (currentBoundInterval1.valueMax - currentBoundInterval1.valueMin < maxDistBetweenLowerAndUpperBound)
  {

    boundListDeriv1.push_back(currentBoundInterval1);
    boundListDeriv2.push_back(currentBoundInterval2);
    // Update maximal recursivity order.
    if (attMaxNbIntervals < maxNbIntervals)
    {
      attMaxNbIntervals = maxNbIntervals;
    }

    return KD_OK;
  }
  else
  {
    // Split interval.
    double uMin = currentBoundInterval1.uMin;
    double uMax = currentBoundInterval1.uMax;
    double uMiddle = 0.5*(uMin+uMax);

    TflicBoundInterval firstIntervalGamma1, firstIntervalGamma2 ;
    TflicBoundInterval secondIntervalGamma1, secondIntervalGamma2;

    // First sub-interval
    firstIntervalGamma2.uMin = uMin ;
    firstIntervalGamma2.uMax = uMiddle ;

    firstIntervalGamma1.uMin = uMin ;
    firstIntervalGamma1.uMax = uMiddle ;

    // Second sub-interval
    secondIntervalGamma2.uMin = uMiddle ;
    secondIntervalGamma2.uMax = uMax ;

    secondIntervalGamma1.uMin = uMiddle ;
    secondIntervalGamma1.uMax = uMax ;

    if ((recursiveBuildBoundLists(2*maxNbIntervals, firstIntervalGamma1, firstIntervalGamma2) == KD_OK) &&
        (recursiveBuildBoundLists(2*maxNbIntervals, secondIntervalGamma1, secondIntervalGamma2) == KD_OK))
    {
      return KD_OK;
    }
    else return KD_ERROR ;
  }

}


// ==============================================================================

ktStatus CflicBoundManagerDefParam::buildBoundLists()
{

  ktStatus status = KD_OK ;

  //initialisation
  TflicBoundInterval boundIntervalGamma1, boundIntervalGamma2 ;


  maxDistBetweenLowerAndUpperBound = sqrt(pow(flatEndCfg.xp-flatStartCfg.xp, 2.0) + pow(flatEndCfg.yp-flatStartCfg.yp, 2.0))/20;
  minNormGamma1 = 2*maxDistBetweenLowerAndUpperBound;

  if (maxDistBetweenLowerAndUpperBound < 10e-8)
  {

    //debug
    //cerr << " FAILURE  ---  maxDistBetweenLowerAndUpperBound < 10e-8 : " << maxDistBetweenLowerAndUpperBound << endl ;

    return KD_ERROR;
  }

  boundIntervalGamma2.uMin = 0 ;
  boundIntervalGamma2.uMax = 1 ;
  boundIntervalGamma2.valueMax = 0 ;
  boundIntervalGamma2.valueMin = 0 ;

  boundIntervalGamma1.uMin = 0 ;
  boundIntervalGamma1.uMax = 1 ;
  boundIntervalGamma1.valueMax = 0 ;
  boundIntervalGamma1.valueMin = 0 ;

  status = recursiveBuildBoundLists(1, boundIntervalGamma1, boundIntervalGamma2) ;

  return status ;

}


// ==============================================================================

double CflicBoundManagerDefParam::computeUpperBoundGamma3()
{
  double maxGamma3;

  double tau1 = flatStartCfg.tau ;
  double tau2 = flatEndCfg.tau ;
  double kap1 = flatStartCfg.kappa ;
  double kap2 = flatEndCfg.kappa ;
  double tauB2 = tau2 - kap2*fabs(attFlatV2) ;

  // ||Gamma2(0)-Gamma1(0)||
  double tabDeriv[6] ;
  flatGamma(&flatEndCfg, -attFlatV2, 2, tabDeriv ) ;
  double normeGamma0 = sqrt(pow(tabDeriv[0]-flatStartCfg.xp, 2.0)+pow(tabDeriv[1]-flatStartCfg.yp,2.0)) ;

  // debug
  //cout << "tau1 : " << tau1 << " tau2 : " << tau2 <<" kap1 : " << kap1 <<" kap2 : " << kap2 <<" tauB2 : " << tauB2 << " normeGamma0 : " << normeGamma0 << endl ;
  //-----------------------------

  maxGamma3 =	fabs(pow(kap1,2)*pow(attFlatV2,3)) +
              fabs(pow(kap2,2)*pow(attFlatV2,3)) +
              3*45/8*pow(attFlatV2, 2) * ( fabs(kap2) * (fabs( tau1 - tauB2 ) + fabs (kap1-kap2) *fabs(attFlatV2) ) + fabs( kap2 - kap1) ) +
              3*17.319*fabs(attFlatV2) * (fabs( tauB2 - tau1 ) + fabs(kap2-kap1) * fabs(attFlatV2) ) +
              60 * ( normeGamma0  + fabs(attFlatV2)*fabs(tauB2-tau1) + fabs(kap2-kap1) * pow(attFlatV2,2)/2)  ;

  return maxGamma3;
}


// ==============================================================================
double CflicBoundManagerDefParam::computeValueGamma2(double u)
{

  double tabDeriv[6] ;

  flatCombination(&flatStartCfg, &flatEndCfg, u, attFlatV2, 2, tabDeriv) ;

  return sqrt(pow(tabDeriv[4], 2)  + pow(tabDeriv[5], 2))  ;

}

// ==============================================================================
double CflicBoundManagerDefParam::computeValueGamma1(double u)
{


  double tabDeriv[6] ;

  flatCombination(&flatStartCfg, &flatEndCfg, u, attFlatV2, 2, tabDeriv) ;

  return sqrt(pow(tabDeriv[2], 2)  + pow(tabDeriv[3], 2))  ;

}











// ==============================================================================
//
//  METHOD OF CLASS CflicArcLengthManager
//
// ==============================================================================



/*****************************************
 PUBLIC METHODS
*****************************************/



CflicArcLengthManager::CflicArcLengthManager(unsigned int nbSampleIntervals,
    CflicDirectPathShPtr inDirectPath):
    attDefaultToArcLength(nbSampleIntervals, 0.0, 1.0), attArcLengthToDefault(nbSampleIntervals)
{
  attNbSampleIntervals = nbSampleIntervals;

  // Build mapping between default parameter and arc-length parameter.
  buildMappingDefaultToArcLength(inDirectPath);
  // Build mapping between arc-length parameter and default parameter.
  buildMappingArcLengthToDefault();
}


// ==============================================================================


double CflicArcLengthManager::defaultParam(double arcLength)
{
  double defaultParam = attArcLengthToDefault.value(arcLength);
  return defaultParam;
}

/*****************************************
 PRIVATE METHODS
*****************************************/

void CflicArcLengthManager::buildMappingDefaultToArcLength(CflicDirectPathShPtr inDirectPath)
{
  double sampleStep = 1.0/attNbSampleIntervals;
  unsigned int rank=0;

  // Set first sample polynomial.
  double valU1 = 0.0;
  attDefaultToArcLength.valueU1(rank, 0.0);
  double valDerivU1 = inDirectPath->normGammaDeriv1(0);
  attDefaultToArcLength.valueDerivU1(rank, valDerivU1);
  double valDerivU2 = inDirectPath->normGammaDeriv1(sampleStep);
  attDefaultToArcLength.valueDerivU2(rank, valDerivU2);
  double mediumValDeriv = inDirectPath->normGammaDeriv1(.5*sampleStep);
  // Simpson integral
  double valU2 = sampleStep*(valDerivU1 + 4*mediumValDeriv + valDerivU2)/6.0;
  attDefaultToArcLength.valueU2(rank, valU2);

  rank++;

  double prevValU2 = valU2;
  double prevValDerivU2 = valDerivU2;

  while (rank < attNbSampleIntervals)
  {
    // Arc length is continuous.
    valU1 = prevValU2;
    attDefaultToArcLength.valueU1(rank, valU1);
    // And even of class C^1
    valDerivU1 = prevValDerivU2;
    attDefaultToArcLength.valueDerivU1(rank, valDerivU1);

    // Integrand value in the middle of the interval.
    mediumValDeriv = inDirectPath->normGammaDeriv1((rank+.5)*sampleStep);
    // Integrand value at end of interval.
    valDerivU2 = inDirectPath->normGammaDeriv1((rank+1)*sampleStep);
    attDefaultToArcLength.valueDerivU2(rank, valDerivU2);
    // Simpson integral.
    valU2 = valU1 + sampleStep*(valDerivU1 + 4*mediumValDeriv + valDerivU2)/6.0;
    attDefaultToArcLength.valueU2(rank, valU2);

    prevValU2 = valU2;
    prevValDerivU2 = valDerivU2;
    rank++;
  }
  attArcLength = prevValU2;
}


// ==============================================================================


void CflicArcLengthManager::buildMappingArcLengthToDefault()
{
  unsigned int rank=0;
  double sampleStep = attArcLength/attNbSampleIntervals;

  //std::vector<CflicPolynomial3>::iterator arcLengthToDefPolyIter = attArcLengthToDefault.begin();
  //std::vector<CflicPolynomial3>::iterator prevArcLengthToDefPolyIter;

  // Fill first interval data
  // interval of definition is [0,sampleStep].
  attArcLengthToDefault.u1(rank, 0);
  attArcLengthToDefault.u2(rank, sampleStep);
  // u(0) = 0
  double valU1 = 0.0;
  attArcLengthToDefault.valueU1(rank, valU1);
  // Find u(sampleStep) by solving equation s(u) = sampleStep.
  double valU2 = findDefaultParameter(sampleStep);
  attArcLengthToDefault.valueU2(rank, valU2);
  // du/ds(0) = 1/(ds/du(0))
  double valDerivU1 = 1.0/attDefaultToArcLength.valueDeriv(0);
  attArcLengthToDefault.valueDerivU1(rank, valDerivU1);
  // du/ds(u2) = 1/(ds/du(s^{-1}(u2)))
  double valDerivU2 = 1.0/attDefaultToArcLength.valueDeriv(valU2);
  attArcLengthToDefault.valueDerivU2(rank, valDerivU2);

  double prevValU2 = valU2;
  double prevValDerivU2 = valDerivU2;
  rank++;

  while (rank < attNbSampleIntervals)
  {

    attArcLengthToDefault.u1(rank, rank*sampleStep);
    attArcLengthToDefault.u2(rank, (rank+1)*sampleStep);

    attArcLengthToDefault.valueU1(rank, prevValU2);
    attArcLengthToDefault.valueDerivU1(rank, prevValDerivU2);

    // Default param at end of interval.
    valU2 = findDefaultParameter((rank+1)*sampleStep);
    attArcLengthToDefault.valueU2(rank, valU2);
    valDerivU2 = 1/attDefaultToArcLength.valueDeriv(valU2);
    attArcLengthToDefault.valueDerivU2(rank, valDerivU2);

    prevValU2 = valU2;
    prevValDerivU2 = valDerivU2;
    rank++;
  }
  attArcLengthToDefault.setDefIntervalBounds();
  attArcLengthToDefault.computeDerivativeBounds();
}

// ==============================================================================


double CflicArcLengthManager::findDefaultParameter(double arcLengthParam)
{
  // If arcLength param is less than 0, return 0.
  if (arcLengthParam <= 0)
  {
    return attDefaultToArcLength.uMin();
  }
  if (arcLengthParam >= attArcLength)
  {
    return attDefaultToArcLength.uMax();
  }

  // Dichotomy
  double middleParam, middleValue;
  double lowerParam = attDefaultToArcLength.uMin();
  double upperParam = attDefaultToArcLength.uMax();
  double lowerValue = attDefaultToArcLength.value(lowerParam);
  double upperValue = attDefaultToArcLength.value(upperParam);

  while (upperParam - lowerParam > 1e-8)
  {
    //middleParam = lowerParam + (arcLengthParam-lowerValue)/(upperValue-lowerValue)*(upperParam-lowerParam);
    middleParam = .5*(lowerParam + upperParam);
    middleValue = attDefaultToArcLength.value(middleParam);

    if (middleValue > arcLengthParam)
    {
      upperParam = middleParam;
      upperValue = middleValue;
    }
    else
    {
      lowerParam = middleParam;
      lowerValue = middleValue;
    }
  }
  return middleParam;
}

// ==============================================================================
//
//  METHOD OF CLASS CflicBoundManagerArcLengthParam
//
// ==============================================================================



/*****************************************
 PUBLIC METHODS
*****************************************/

void CflicBoundManagerArcLengthParam::getBoundsNormDgammaOverDs(double sMin, double sMax,
    double& valueMin, double& valueMax)
{
  getBoundsOnInterval(boundVectorDeriv1, sMin, sMax, valueMin, valueMax);
}

void CflicBoundManagerArcLengthParam::getBoundsNormD2gammaOverDs2(double sMin, double sMax,
    double& valueMin, double& valueMax)
{
  getBoundsOnInterval(boundVectorDeriv2, sMin, sMax, valueMin, valueMax);
}
/*****************************************
 PRIVATE METHODS
*****************************************/

ktStatus CflicBoundManagerArcLengthParam::buildBoundVectors(CflicDirectPathShPtr inDirectPath)
{
  double sampleStep = attArcLength/attNbSampleIntervals;
  unsigned int iterNb;
  double minGamma2, maxGamma2;
  double minGamma1, maxGamma1;
  double maxGamma3;

  // Iterator over vector of degree 3 polynomials defining mapping from arc length to default param.
  std::vector<CflicPolynomial3>::const_iterator arcLengthToDefaultMappingIter =
    inDirectPath->attArcLengthManager->mappingArcLengthToDefIterBegin();

  for(iterNb = 0; iterNb < attNbSampleIntervals; iterNb++)
  {
    double sMin = iterNb*sampleStep;
    double sMax = (iterNb+1)*sampleStep;
    // Set definition interval.
    boundVectorDeriv1[iterNb].uMin = sMin;
    boundVectorDeriv1[iterNb].uMax = sMax;
    boundVectorDeriv2[iterNb].uMin = sMin;
    boundVectorDeriv2[iterNb].uMax = sMax;

    double uMin = inDirectPath->attArcLengthManager->defaultParam(sMin);
    double uMax = inDirectPath->attArcLengthManager->defaultParam(sMax);
    // Get maximal norm of gamma first derivative over interval.
    inDirectPath->attBoundManagerDefParam->getBoundsNormGamma1(uMin, uMax, minGamma1, maxGamma1);
    // Get maximal norm of gamma second derivative over interval.
    inDirectPath->attBoundManagerDefParam->getBoundsNormGamma2(uMin, uMax, minGamma2, maxGamma2);
    // Get maximal norm of gamma second derivative over interval.
    maxGamma3 = inDirectPath->attBoundManagerDefParam->getBoundsNormGamma3();

    // Get maximal absolute value of du/ds.
    double maxAbsDuOverDs = arcLengthToDefaultMappingIter->maxAbsDeriv1();
    // Get maximal absolute value of d^2u/ds^2.
    double maxAbsD2uOverDs2 = arcLengthToDefaultMappingIter->maxAbsDeriv2();
    // Get maximal absolute value of d^3u/ds^3.
    double maxAbsD3uOverDs3 = arcLengthToDefaultMappingIter->maxAbsDeriv3();

    attMaxD3gammaOverDs3 =
      maxGamma3*pow(maxAbsDuOverDs, 2.0) +
      3*maxGamma2*maxAbsD2uOverDs2*maxAbsDuOverDs +
      maxGamma1*maxAbsD3uOverDs3;

    //
    // Computation of dgamma/ds and d^2gamma/ds^2 norms at sMin
    //

    // First and second derivatives of gamma (wrt u) at uMin
    double xDgammaOverDu, yDgammaOverDu, xD2gammaOverDu2, yD2gammaOverDu2;
    if (inDirectPath->gammaDeriv1and2(uMin, xDgammaOverDu, yDgammaOverDu,
                                      xD2gammaOverDu2,yD2gammaOverDu2) != KD_OK)
    {
      cerr << "CflicBoundManagerArcLengthParam::buildBoundVectors: unable to compute gamma derivatives"
      << endl;
      return KD_ERROR;
    }
    // First and second derivatives of u wrt s at sMin.
    double duOverDs = arcLengthToDefaultMappingIter->valueDeriv(sMin);
    double d2uOverDs2 = arcLengthToDefaultMappingIter->valueDeriv2(sMin);

    // Vector dgamma/ds at Smin
    double xDgammaOverDs = xDgammaOverDu*duOverDs;
    double yDgammaOverDs = yDgammaOverDu*duOverDs;

    // Norm of vector dgamma/ds at Smin
    double normDgammaOverDsSmin = sqrt(pow(xDgammaOverDs ,2.0) + pow(yDgammaOverDs ,2.0));

    // Vector d^2gamma/ds^2 at Smin
    double xD2gammaOverDs2 = pow(duOverDs, 2.0)*xD2gammaOverDu2 + d2uOverDs2*xDgammaOverDu;
    double yD2gammaOverDs2 = pow(duOverDs, 2.0)*yD2gammaOverDu2 + d2uOverDs2*yDgammaOverDu;

    // Norm of vector d^2gamma/ds^2 at Smin
    double normD2gammaOverDs2Smin = sqrt(pow(xD2gammaOverDs2, 2.0) + pow(yD2gammaOverDs2, 2.0));

    //
    // Computation of of dgamma/ds and d^2gamma/ds^2 norms at sMax
    //

    // First and second derivatives of gamma (wrt u) at uMax
    if (inDirectPath->gammaDeriv1and2(uMax, xDgammaOverDu, yDgammaOverDu,
                                      xD2gammaOverDu2,yD2gammaOverDu2) != KD_OK)
    {
      cerr << "CflicBoundManagerArcLengthParam::buildBoundVectors: unable to compute gamma derivatives"
      << endl;
      return KD_ERROR;
    }
    // First and second derivatives of u wrt s at sMax.
    duOverDs = arcLengthToDefaultMappingIter->valueDeriv(sMax);
    d2uOverDs2 = arcLengthToDefaultMappingIter->valueDeriv2(sMax);

    // Vector dgamma/ds at Smax
    xDgammaOverDs = xDgammaOverDu*duOverDs;
    yDgammaOverDs = yDgammaOverDu*duOverDs;

    // Norm of vector dgamma/ds at Smax
    double normDgammaOverDsSmax = sqrt(pow(xDgammaOverDs ,2.0) + pow(yDgammaOverDs ,2.0));

    xD2gammaOverDs2 = pow(duOverDs, 2.0)*xD2gammaOverDu2 + d2uOverDs2*xDgammaOverDu;
    yD2gammaOverDs2 = pow(duOverDs, 2.0)*yD2gammaOverDu2 + d2uOverDs2*yDgammaOverDu;

    // Norm of vector d^2gamma/ds^2 at Smax
    double normD2gammaOverDs2Smax = sqrt(pow(xD2gammaOverDs2, 2.0) + pow(yD2gammaOverDs2, 2.0));

    //
    // Lower and upper bounds of second derivative.
    //

    if (computeFunctionBoundsFromDerivativeUpperBound(boundVectorDeriv2[iterNb], normD2gammaOverDs2Smin,
        normD2gammaOverDs2Smax, attMaxD3gammaOverDs3) != KD_OK)
    {
      return KD_ERROR;
    }

    //
    // Lower and upper bounds of first derivative.
    //

    double maxD2gammaOverds2 = boundVectorDeriv2[iterNb].valueMax;

    if (computeFunctionBoundsFromDerivativeUpperBound(boundVectorDeriv1[iterNb], normDgammaOverDsSmin,
        normDgammaOverDsSmax, maxD2gammaOverds2) != KD_OK)
    {
      return KD_ERROR;
    }
    // Test that minimum value of norm of gamma first derivative is positive.
    if (boundVectorDeriv1[iterNb].valueMin <= 0.5)
    {
      return KD_ERROR;
    }
    arcLengthToDefaultMappingIter++;
  }
  return KD_OK;
}

void CflicBoundManagerArcLengthParam::getBoundsOnInterval(const std::vector<TflicBoundInterval>& boundIntVector,
    double sMin, double sMax,
    double& valueMin, double& valueMax)
{
  unsigned int sampleIntervalId;
  unsigned int sampleIntervalIdSmin = (unsigned int)floor(sMin*attNbSampleIntervals/attArcLength);
  unsigned int sampleIntervalIdSmax = (unsigned int)floor(sMax*attNbSampleIntervals/attArcLength);

  if (sampleIntervalIdSmin == attNbSampleIntervals)
  {
    sampleIntervalIdSmin = attNbSampleIntervals-1;
  }
  if (sampleIntervalIdSmax == attNbSampleIntervals)
  {
    sampleIntervalIdSmax = attNbSampleIntervals-1;
  }
  valueMin = boundIntVector[sampleIntervalIdSmin].valueMin;
  valueMax = boundIntVector[sampleIntervalIdSmin].valueMax;

  for (sampleIntervalId = sampleIntervalIdSmin;
       sampleIntervalId <= sampleIntervalIdSmax;
       sampleIntervalId++)
  {
    if (valueMin > boundIntVector[sampleIntervalId].valueMin)
    {
      valueMin = boundIntVector[sampleIntervalId].valueMin;
    }
    if (valueMax < boundIntVector[sampleIntervalId].valueMax)
    {
      valueMax = boundIntVector[sampleIntervalId].valueMax;
    }
  }
}


// ==============================================================================
//
//  METHOD OF CLASS CflicDirectPath
//
// ==============================================================================



/*****************************************
 PUBLIC METHODS
*****************************************/

// =========================================================================================

// ==============================================================================
//
//  METHOD OF CLASS CflicPolynomial3
//
// ==============================================================================

CflicDirectPath::~CflicDirectPath()
{
  //------debug
  //cout << "flat DP destructor de x1= " << flatStartCfg.xp << " \ty1= " <<  flatStartCfg.yp  <<" \tT1= " <<  flatStartCfg.tau
  //     <<" \tK1= " <<  flatStartCfg.kappa  << endl
  //     << "\t\tA x2= " <<flatEndCfg.xp << " \ty2= " <<  flatEndCfg.yp<< " \tT2= "
  //     <<  flatEndCfg.tau << " \tK2= " <<  flatEndCfg.kappa<< endl << endl ;
  //--------------------------------------
}

// =========================================================================================

CflicDirectPathShPtr CflicDirectPath::create(const CkwsConfig &inStartCfg,
    const CkwsConfig &inEndCfg,
    const CkwsSteeringMethodShPtr &inSteeringMethod,
    bool inOriented)
{

  CflicDirectPath* pathPtr = new CflicDirectPath(inStartCfg, inEndCfg,inSteeringMethod);
  CflicDirectPathShPtr pathShPtr(pathPtr);
  CflicDirectPathWkPtr pathWkPtr(pathShPtr) ;

  // Create derivative bound manager for default parameter.
  if (pathShPtr)
  {
    CflicBoundManagerDefParamShPtr boundManager(new CflicBoundManagerDefParam(pathShPtr,inStartCfg, inEndCfg));
    pathShPtr->attBoundManagerDefParam = boundManager;
    if ( (pathShPtr->attBoundManagerDefParam->buildBoundLists()) == KD_ERROR)
    {
      pathShPtr.reset();
    }
  }

  // Create arc length manager (shared pointer) that compute the mappings between default and arc-length
  // parameter and inverse mapping.
  if (pathShPtr)
  {
    CflicArcLengthManagerShPtr arcLengthManager(new CflicArcLengthManager(pathShPtr->attBoundManagerDefParam->attMaxNbIntervals,pathShPtr));

    pathShPtr->attArcLengthManager = arcLengthManager;
  }

  // Create derivative bound manager for arc-length parameter.
  if (pathShPtr)
  {
    CflicBoundManagerArcLengthParamShPtr
    boundManagerAL(new CflicBoundManagerArcLengthParam(pathShPtr->attBoundManagerDefParam->attMaxNbIntervals,
                   pathShPtr->attArcLengthManager->attArcLength,
                   pathShPtr));
    pathShPtr->attBoundManagerArcLengthParam = boundManagerAL;
    if ((pathShPtr->attBoundManagerArcLengthParam->buildBoundVectors(pathShPtr)) == KD_ERROR)
    {
      pathShPtr.reset();
    }
  }

  // Init should be at the end since CkwsDirectPath::init() calls computePrivateLength that is defined by
  // attArcLengthManager
  if (pathShPtr) {
    if (pathPtr->init(pathWkPtr, inOriented) != KD_OK) {
      pathShPtr.reset()	;
    }
  }

  if (!pathShPtr)
  {
    //cerr << " \\\\ CflicDirectPath::create failed //// " << endl ;
    //cerr << "init : " << inStartCfg << endl;
    //cerr << "end : "  << inEndCfg << endl ;
  }
  else
  {
    // debug
    // cout << "================= CflicDirectPath::create succeeded ===============================" << endl;
    // cout << "init : " << inStartCfg << endl;
    // cout << "end : "  << inEndCfg << endl ;
    // -----------
  }

  return pathShPtr ;
}

// =========================================================================================

CflicDirectPathShPtr CflicDirectPath::createCopy(const CflicDirectPathConstShPtr &inFlicDirectPath)
{

  if(inFlicDirectPath != NULL)
  {
    CflicDirectPath* pathPtr = new CflicDirectPath(*inFlicDirectPath) ;
    CflicDirectPathShPtr pathShPtr(pathPtr) ;
    CflicDirectPathWkPtr pathWkPtr(pathShPtr) ;

    if(pathPtr->init(pathWkPtr) != KD_OK)
    {
      pathShPtr.reset() ;
      return pathShPtr;
    }
    // Copy of attributes attBoundManagerDefParam, attArcLengthManager, attBoundManagerArcLengthParam.
    if (inFlicDirectPath->attBoundManagerDefParam)
    {
      pathShPtr->attBoundManagerDefParam = inFlicDirectPath->attBoundManagerDefParam;
    }
    else
    {
      cerr << "CflicDirectPath::createCopy: attBoundManagerDefParam does not exist in the copied direct path." << endl;
    }

    if (inFlicDirectPath->attArcLengthManager)
    {
      pathShPtr->attArcLengthManager = inFlicDirectPath->attArcLengthManager;
    }
    else
    {
      cerr << "CflicDirectPath::createCopy: attArcLengthManager does not exist in the copied direct path." << endl;
    }

    if (inFlicDirectPath->attBoundManagerArcLengthParam)
    {
      pathShPtr->attBoundManagerArcLengthParam = inFlicDirectPath->attBoundManagerArcLengthParam;
    }
    else
    {
      cerr << "CflicDirectPath::createCopy: attBoundManagerArcLengthParam does not exist in the copied direct path." << endl;
    }

    return pathShPtr;

  }
  else return CflicDirectPathShPtr() ;

}

// =========================================================================================

CkwsAbstractPathShPtr CflicDirectPath::clone() const
{

  return CflicDirectPath::createCopy(m_weakPtr.lock());

}

/*****************************************
 PROTECTED METHODS
*******************************************/


// =========================================================================================

CflicDirectPath::CflicDirectPath(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg,
                                 const CkwsSteeringMethodShPtr &inSteeringMethod) :
    CkwsDirectPath(inStartCfg, inEndCfg, inSteeringMethod)
{
  //configuration convertion
  flatStartCfg.kappa = inStartCfg.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  flatStartCfg.xp = inStartCfg.dofValue(X_COORD) ;  // x
  flatStartCfg.yp = inStartCfg.dofValue(Y_COORD) ;  // y
  flatStartCfg.tau = inStartCfg.dofValue(RZ_COORD) ; // theta

  flatEndCfg.kappa = inEndCfg.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  flatEndCfg.xp = inEndCfg.dofValue(X_COORD) ;  // x
  flatEndCfg.yp = inEndCfg.dofValue(Y_COORD) ;  // y
  flatEndCfg.tau = inEndCfg.dofValue(RZ_COORD) ; // theta

  attFlatV2 = computeFlatV2(&flatStartCfg, &flatEndCfg);

  // debug
  //cout << "Constructor by configurations" << endl;
  // ---------------------
}

CflicDirectPath::CflicDirectPath(const CflicDirectPath &inDirectPath):CkwsDirectPath(inDirectPath)
{
  //configuration convertion
  flatStartCfg.kappa = m_start.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  flatStartCfg.xp = m_start.dofValue(X_COORD) ;  // x
  flatStartCfg.yp = m_start.dofValue(Y_COORD) ;  // y
  flatStartCfg.tau = m_start.dofValue(RZ_COORD) ; // theta

  flatEndCfg.kappa = m_end.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  flatEndCfg.xp = m_end.dofValue(X_COORD) ;  // x
  flatEndCfg.yp = m_end.dofValue(Y_COORD) ;  // y
  flatEndCfg.tau = m_end.dofValue(RZ_COORD) ; // theta

  attFlatV2 = inDirectPath.flatV2();

  // debug
  // cout << "Copy constructor" << endl;
  // ------------------------
}

// =========================================================================================

ktStatus CflicDirectPath::init(const CflicDirectPathWkPtr &inWeakPtr)
{

  ktStatus success = CkwsDirectPath::init(inWeakPtr) ;

  if (KD_OK == success) m_weakPtr = inWeakPtr;

  return success ;

}

// =========================================================================================

ktStatus CflicDirectPath::init(const CflicDirectPathWkPtr &inWeakPtr, bool inOriented)
{

  ktStatus success = CkwsDirectPath::init(inWeakPtr) ;

  if (inOriented && attFlatV2 <= 0)
  {
    success = KD_ERROR;
  }

  if (KD_OK == success) m_weakPtr = inWeakPtr;

  return success ;

}

// =========================================================================================

double CflicDirectPath::computePrivateLength()const
{

  //debug
  //cout << "enter in computePrivateLength " << endl ;
  //------------------------
  
  return attArcLengthManager->attArcLength;
}

// =========================================================================================

void CflicDirectPath::interpolateDefaultParam(double u, CkwsConfig & outCfg) const
{

  KWS_PRECONDITION( m_start.size() == device()->countDofs() );
  KWS_PRECONDITION( m_start.size() == outCfg.size() );

#if 0
  double theta, K;
#endif
  int deriv_order = 2 ;
  double Tab_gamma[6] ;
  TflatConfig f1, f2 ;

  f1.kappa = m_start.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  f1.xp = m_start.dofValue(X_COORD) ;  // x
  f1.yp = m_start.dofValue(Y_COORD) ;  // y
  f1.tau = m_start.dofValue(RZ_COORD) ; // theta


  f2.kappa = m_end.dofValue(CURV_COORD) ; // Kappa (dof fictif)
  f2.xp = m_end.dofValue(X_COORD) ; //  x
  f2.yp = m_end.dofValue(Y_COORD) ; //  y
  f2.tau = m_end.dofValue(RZ_COORD); // theta


  // compute the Tflatconfiguration corresponding to the i
  flatCombination(&f1, &f2, u, attFlatV2, deriv_order, Tab_gamma) ;

  int forward = (attFlatV2 > 0 ? true : false);
  TflatConfig flatConfig;


  flatConvDerivFlatconfig(Tab_gamma, &flatConfig, forward);
#if 0 // Florent // a enlever
  // compute theta
  theta = atan2 (Tab_gamma[3],Tab_gamma[2]) ;

  // compute Kappa
  // K = (y''*x'-x''*y')/sqrt(pow(x'*x' +y'*y', 3)) ;
  K = (Tab_gamma[5]*Tab_gamma[2]-Tab_gamma[4]*Tab_gamma[3])/sqrt(pow(Tab_gamma[2]*Tab_gamma[2] +Tab_gamma[3]*Tab_gamma[3], 3)) ;
#endif

  outCfg.beginChange();

  outCfg.dofValue(CURV_COORD, flatConfig.kappa) ; // kappa
  outCfg.dofValue(X_COORD, flatConfig.xp) ; //  px
  outCfg.dofValue(Y_COORD, flatConfig.yp) ; //  py
  outCfg.dofValue(Z_COORD, 0) ; //  pz
  outCfg.dofValue(RX_COORD, 0) ; //  
  outCfg.dofValue(RY_COORD, 0) ; //  
  outCfg.dofValue(RZ_COORD, flatConfig.tau ) ;       //  theta


  outCfg.endChange() ;


}

void CflicDirectPath::interpolate(double s, CkwsConfig & outCfg) const
{

  KWS_PRECONDITION( m_start.size() == device()->countDofs() );
  KWS_PRECONDITION( m_start.size() == outCfg.size() );

  double u = attArcLengthManager->defaultParam(s);
  interpolateDefaultParam(u, outCfg);
}


// ==========================================================================================

void CflicDirectPath::maxAbsoluteDerivDefaultParam(double inFrom, double inTo, std::vector<double> & outVectorDeriv) const
{
  //debug
  //cout << " - MAX ABSOLUTE DERIVATE - " << endl ;
  //-----------------------------------------

  KWS_PRECONDITION( m_start.size() == device()->countDofs() );

  CkwsDeviceShPtr dev(device());
  outVectorDeriv.resize(dev->countDofs()); // that causes error with 2.04
  // outVectorDeriv.resize(m_start.size());

  double mini2 = 1 , max2 =1 , mini1 =1   , max1 =1 ;
  attBoundManagerDefParam->getBoundsOnInterval(attBoundManagerDefParam->getBoundListDeriv2(), inFrom,  inTo,  mini2,  max2) ;
  attBoundManagerDefParam->getBoundsOnInterval(attBoundManagerDefParam->getBoundListDeriv1(), inFrom,  inTo,  mini1,  max1) ;
  double max3 = attBoundManagerDefParam->getUpperBoundGamma3();


  //debug
  // cout << "De " <<  inFrom << " a " << inTo << " maxDerivX : " << max1 << "\t maxDerivY : " << max1
  //<< "\t maxDerivT : " << max2/mini1 << " \tmax2 :" << max2 << " \tmini 1 :" << mini1  << endl ;
  //-------------------------------------------------

  outVectorDeriv[0] = max3/pow(mini1,2) + 3*max2/pow(mini1,3); // kappa
  outVectorDeriv[1] = max1  ; // x
  outVectorDeriv[2] = max1  ; // y
  outVectorDeriv[3] = max2/mini1 ; // theta

  //debug
  //cout << " - MAX ABSOLUTE DERIVATE - " << endl ;
  //------------------------------

}

// ==========================================================================================

void CflicDirectPath::maxAbsoluteDerivative(double inFrom, double inTo, std::vector<double> & outVectorDeriv) const
{
  KWS_PRECONDITION( m_start.size() == device()->countDofs() );

  CkwsDeviceShPtr dev(device());
  outVectorDeriv.resize(dev->countDofs()); // that causes error with 2.04
  // outVectorDeriv.resize(m_start.size());

  double max3 = attBoundManagerArcLengthParam->maxD3gammaOverDs3();
  double min2 = 1 , max2 =1 , min1 =1   , max1 =1 ;

  attBoundManagerArcLengthParam->getBoundsNormDgammaOverDs(inFrom, inTo, min1, max1);
  attBoundManagerArcLengthParam->getBoundsNormD2gammaOverDs2(inFrom, inTo, min2, max2);

  // Device is now a freeflyer. 
  // Set non planar dof values to very small value (0 could lead to troubles).

  outVectorDeriv[0] = max3/pow(min1,2) + 3*max2/pow(min1,3); // kappa
  outVectorDeriv[1] = max1  ;     // x
  outVectorDeriv[2] = max1  ;     // y
  outVectorDeriv[3] = 1e-10;        // z
  outVectorDeriv[4] = 1e-10;        // Rx
  outVectorDeriv[5] = 1e-10;        // Ry
  outVectorDeriv[6] = max2/min1 ; // Rz
}


// ==========================================================================================


double CflicDirectPath::normGammaDeriv1(double u)
{
  KWS_PRECONDITION( m_start.size() == device()->countDofs() );
  KWS_PRECONDITION( m_start.size() == outCfg.size() );


  double v2;
  int deriv_order = 2 ;
  double Tab_gamma[6] ;
  TflatConfig f1, f2 ;
  double normDeriv;

  f1.kappa = m_start.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  f1.xp = m_start.dofValue(X_COORD) ;  // x
  f1.yp = m_start.dofValue(Y_COORD) ;  // y
  f1.tau = m_start.dofValue(RZ_COORD) ; // theta


  f2.kappa = m_end.dofValue(CURV_COORD) ; // Kappa (dof fictif)
  f2.xp = m_end.dofValue(X_COORD) ; //  x
  f2.yp = m_end.dofValue(Y_COORD) ; //  y
  f2.tau = m_end.dofValue(RZ_COORD); // theta


  // compute the Tflatconfiguration corresponding to the i
  v2 = attFlatV2;
  flatCombination(&f1, &f2, u, attFlatV2, deriv_order, Tab_gamma) ;

  normDeriv = sqrt(pow(Tab_gamma[2],2.0) + pow(Tab_gamma[3],2.0));

  return normDeriv;
}

// ==========================================================================================


ktStatus CflicDirectPath::gammaDeriv1and2(double u, double& xGamma_1, double& yGamma_1,
    double& xGamma_2, double& yGamma_2)
{
  KWS_PRECONDITION( m_start.size() == device()->countDofs() );
  KWS_PRECONDITION( m_start.size() == outCfg.size() );

  int deriv_order = 2 ;
  double Tab_gamma[6] ;
  TflatConfig f1, f2 ;

  f1.kappa = m_start.dofValue(CURV_COORD) ; //Kappa (dof fictif)
  f1.xp = m_start.dofValue(X_COORD) ;  // x
  f1.yp = m_start.dofValue(Y_COORD) ;  // y
  f1.tau = m_start.dofValue(RZ_COORD) ; // theta


  f2.kappa = m_end.dofValue(CURV_COORD) ; // Kappa (dof fictif)
  f2.xp = m_end.dofValue(X_COORD) ; //  x
  f2.yp = m_end.dofValue(Y_COORD) ; //  y
  f2.tau = m_end.dofValue(RZ_COORD); // theta


  // compute the Tflatconfiguration corresponding to the i
  flatCombination(&f1, &f2, u, attFlatV2, deriv_order, Tab_gamma) ;

  xGamma_1 = Tab_gamma[2];
  yGamma_1 = Tab_gamma[3];
  xGamma_2 = Tab_gamma[4];
  yGamma_2 = Tab_gamma[5];

  return KD_OK;
}

/****************************************************
 EXTERN FUNCTION USE TO COMPUTE FLAT INTERPOLATION
****************************************************/

//=========================================================================================

double CflicDirectPath::computeFlatV2(TflatConfig *fconf1, TflatConfig *fconf2)
{
  double O1M1_x, O1M1_y, O1M2_x, O1M2_y, v2;

  double teta, kappa, Phi, cosinus;
  double norme_O1M1, norme_O1M2, scalaire_O1M1_O1M2;

  teta = fconf1->tau;
  kappa = fconf1->kappa;

  /* First canonical curve is a circle */
  if(fabs(kappa) > NEAR_ZERO)
  {

    O1M1_x = sin(teta)/kappa;
    O1M1_y = -cos(teta)/kappa;

    O1M2_x = fconf2->xp - fconf1->xp + sin(teta)/kappa;
    O1M2_y = fconf2->yp - fconf1->yp - cos(teta)/kappa;

    scalaire_O1M1_O1M2 = O1M1_x*O1M2_x + O1M1_y*O1M2_y;
    norme_O1M1 = sqrt(O1M1_x*O1M1_x + O1M1_y*O1M1_y);
    norme_O1M2 = sqrt(O1M2_x*O1M2_x + O1M2_y*O1M2_y);
    cosinus = scalaire_O1M1_O1M2 / (norme_O1M1*norme_O1M2);
    if (fabs(cosinus) >= 1)
    {
      /* kappa is in fact very small :
      return the same value as in the line case
      */
      v2 = (fconf2->xp - fconf1->xp)*cos(teta) +
           (fconf2->yp - fconf1->yp)*sin(teta);
      return(v2);
    }
    else
    {
      Phi = acos(scalaire_O1M1_O1M2 / (norme_O1M1*norme_O1M2));
    }
    if((O1M2_x*cos(teta) + O1M2_y*sin(teta)) < 0.0) Phi *= -1.0;

    v2 = Phi/fabs(kappa);
    return(v2);
  }
  /* La premiere conf definie une droite */
  else
  {
    v2 = (fconf2->xp - fconf1->xp)*cos(teta) +
         (fconf2->yp - fconf1->yp)*sin(teta);
    return(v2);
  }
}


// =========================================================================================

static void flatGamma(CflicDirectPath::TflatConfig *fconf, double s, double deriv_order, double *Tab_gamma)
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

static void flatAlpha(double u, int deriv_order, double *Tab_alpha)
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

static void flatCombination(CflicDirectPath::TflatConfig *fconf_initPt,
			    CflicDirectPath::TflatConfig *finalFlatConfPt, double u,
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


static double angleLimit(double angle)
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


static void flatConvDerivFlatconfig(double *tabDeriv, CflicDirectPath::TflatConfig *fconf, bool forward)
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


static ktStatus computeFunctionBoundsFromDerivativeUpperBound(TflicBoundInterval& boundInterval,
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
