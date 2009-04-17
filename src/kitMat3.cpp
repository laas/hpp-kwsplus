/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Developed by Florent Lamiraux (LAAS-CNRS)
 
*/

#include "kwsPlus/util/kitMat3.h"

#define SIGN(a) (((a)>0)?(1):(-1))

// ==========================================================================


CkitMat3::CkitMat3()
{
        attCoefficients[0][0] = 1;
        attCoefficients[0][1] = 0;
        attCoefficients[0][2] = 0;
        attCoefficients[0][0] = 0;
        attCoefficients[0][1] = 1;
        attCoefficients[0][2] = 0;
        attCoefficients[0][0] = 0;
        attCoefficients[0][1] = 0;
        attCoefficients[0][2] = 1;
}

// ==========================================================================


CkitMat3::CkitMat3(const CkitMat3& inMat)
{
        for(unsigned int i=0;  i<3; i++) {
                for(unsigned int j=0;  j<3; j++) {
                        attCoefficients[i][j] = inMat(i,j);
                }
        }
}

// ==========================================================================

CkitMat3::CkitMat3(const double inCoef[3][3])
{
        for(unsigned int i=0; i<3; i++) {
                for(unsigned int j=0;  j<3; j++) {
                        attCoefficients[i][j] = inCoef[i][j];
                }
        }
}

// ==========================================================================

double& CkitMat3::operator()(const unsigned int i, const unsigned int j)
{
        KIT_PRECONDITION((i>=0)&&(i<3));
        KIT_PRECONDITION((j>=0)&&(j<3));

        return attCoefficients[i][j];
}

// ==========================================================================

double CkitMat3::operator()(const unsigned int i, const unsigned int j) const
{
        KIT_PRECONDITION((i>=0)&&(i<3));
        KIT_PRECONDITION((j>=0)&&(j<3));

        return attCoefficients[i][j];
}

// ==========================================================================

CkitMat3 CkitMat3::operator*(const CkitMat3& inMat) const
{
        CkitMat3 outMat;

        for (unsigned int i=0; i<3; i++) {
                for (unsigned int k=0; k<3; k++) {
                        outMat.attCoefficients[i][k] = 0;
                        for (unsigned int j=0; j<3; j++) {
                                outMat.attCoefficients[i][k] += attCoefficients[i][j]*inMat.attCoefficients[j][k];
                        }
                }
        }
        return outMat;
}

// ==========================================================================

CkitVect3 CkitMat3::operator*(const CkitVect3& inVect) const
{
        double vectCoef[3];

        for (unsigned int i=0; i<3; i++) {
                vectCoef[i] = 0;
                for (unsigned int j=0; j<3; j++) {
                        vectCoef[i] += attCoefficients[i][j] * inVect[j];
                }
        }

        CkitVect3 outVect(vectCoef[0], vectCoef[1], vectCoef[2]);
        return outVect;
}

// ==========================================================================

CkitMat3 CkitMat3::inverse(const CkitMat3& inMat) const
{
        CkitMat3 invMat;

        for (unsigned int i=0; i<3; i++) {
                for (unsigned int j=0; j<3; j++) {
                        invMat.attCoefficients[i][j] = attCoefficients[j][i];
                }
        }
        return invMat;
}

// ==========================================================================

void CkitMat3::mat3ToBryant(double &Rx, double &Ry, double &Rz)
{


        Rz = atan2( - attCoefficients[0][1],attCoefficients[0][0]);
        double cz = cos(Rz);
        double sz = sin(Rz);
        Rx = atan2( -attCoefficients[1][2], attCoefficients[2][2]);
        if (fabs(Rx) > M_PI/2) {    // keep -M_PI/2 < qx < M_PI/i
                Rz = Rz - SIGN(Rz)*M_PI;
                Rx = Rx - SIGN(Rx)*M_PI;
                cz = -cz;
                sz = -sz;
        }
        Ry = atan2(  attCoefficients[0][2] , attCoefficients[0][0]*cz - attCoefficients[0][1]*sz);


}

// ==========================================================================


void CkitMat3::bryantToMat3(double a, double b, double c)
{

        double g = cos(b)*cos(c);
        double h = -cos(b) * sin(c) ;
        double i =   sin(b) ;
        double j = sin(c)*cos(a) + cos(c)*sin(a)*sin(b) ;
        double k = cos(c)*cos(a) - sin(a)*sin(b)*sin(c) ;
        double l = -sin(a)*cos(b) ;
        double m = -cos(a)*sin(b)*cos(c)+sin(a)*sin(c) ;
        double n = cos(a)*sin(b)*sin(c)+sin(a)*cos(c) ;
        double o = cos(a)*cos(b) ;

        attCoefficients[0][0] = g ;
        attCoefficients[0][1] = h  ;
        attCoefficients[0][2] = i  ;
        attCoefficients[1][0] = j ;
        attCoefficients[1][1] = k ;
        attCoefficients[1][2] = l ;
        attCoefficients[2][0] = m  ;
        attCoefficients[2][1] = n ;
        attCoefficients[2][2] = o  ;

}

// ==========================================================================

CkitMat4 CkitMat3::mat3ToMat4()
{

        CkitMat4 inMat4 ;

        inMat4.identity() ;

        for(unsigned int i=0;  i<3; i++) {
                for(unsigned int j=0;  j<3; j++) {
                        inMat4.setComponent(i, j, attCoefficients[i][j]);
                }
        }

        return inMat4 ;
}

// ==========================================================================

void CkitMat3::mat4ToMat3(const CkitMat4 inMat4)
{

        for(unsigned int i=0; i<3; i++) {
                for(unsigned int j=0;  j<3; j++) {
                        attCoefficients[i][j] = inMat4.getComponent(i,j);
                }
        }

}


