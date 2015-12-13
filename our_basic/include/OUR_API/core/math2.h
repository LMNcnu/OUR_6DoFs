/**
*@file math2.h
*@brief Mathematic functions
*
* Extenstion of mathematic functions in cmath.
*
*@author Smokie Robotics, Inc. US.
*@version 1.0.0
*@date 10-09-2014
*/

#ifndef MATH_H_
#define MATH_H_

#include <cmath>

/*!
 \brief Compute the sign of input

 \param x Input value
 \return double Resultant sign
*/
double SIGN(double x){
	return (x >= 0.0f) ? +1.0f : -1.0f;
}

/*!
 \brief Compute the distance

 \param a Input
 \param b Input
 \param c Input
 \param d Input
 \return double Distance
*/
double NORM(double a, double b, double c, double d){
	return sqrt(a * a + b * b + c * c + d * d);
}

/*!
 \brief Section the number with the precision of 0.000001

 \param num Input number
 \return double Resultant value
*/
double SectionNum(double num){
	return int(num*1000000+0.5f)*0.000001f;
}

/*!
 \brief Section the number with the precision of 0.0001

 \param num Input numebr
 \return double Resultant value
*/
double SectionFourNum(double num){
     return int(num*10000+0.5f)*0.0001f;
}
#endif //math
