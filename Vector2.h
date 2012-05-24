/*
 *  Vector2.h
 *  Description:
 *      This file provides a two dimensional vector class which 
 *      Supports the standard 2-vector operations.
 *
 *  Created by krs9249 on Sat Mar 29 2003.
 *  Copyright (c) 2003 __MyCompanyName__. All rights reserved.
 *  Log:
 *    $Log: Vector2.h,v $
 *    Revision 1.2  2003/04/18 21:50:22  krs9249
 *    added documentation.
 *
 */

#ifndef VECTOR2_H
#define VECTOR2_H
#include <math.h>
class Vector2
{
public:
    double x,y;
    Vector2():x(0),y(0){};
    Vector2( double angle ):x(cos( angle )),y( sin ( angle )){};
    Vector2(  double xi, double yi ):x(xi),y(yi){};
    Vector2& operator +=( const Vector2& in){ x+=in.x; y+=in.y; return *this; };
    Vector2& operator -=( const Vector2& in){ x-=in.x; y-=in.y; return *this; };
    double dot( const Vector2 & in ) const{ return x*in.x + y*in.y;};
    double mag() const{return sqrt(dot( *this ));};
    double mag2() const{return (dot( *this ));};
    Vector2 s()const {return Vector2( -y,x );};
    Vector2 u()const { return *this/mag();}
    Vector2 operator +( const Vector2& in) const {  return Vector2( x+in.x,y+in.y); };
    const Vector2 operator -( const Vector2& in) const{  return Vector2( x-in.x,y-in.y); };
    const Vector2 operator *( const double s ) const{ return Vector2( s*x, s*y);};
    const Vector2 operator /( const double s ) const{ return Vector2( x/s, y/s);};
    
};
inline Vector2 operator * ( double s,  const Vector2& in ) { return Vector2( s*in.x, s*in.y);}



#endif

