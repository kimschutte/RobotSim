/*
 *  Entity.cpp
 *  RobotSim
 *
 *  Created by krs9249 on Sun Mar 30 2003.
 *  Copyright (c) 2003. All rights reserved.
 *
 */

#include "Entity.h"
#include <iostream.h>

//*********************************************************************
vector<int>* Entity::getLineZones( Vector2 p1, Vector2 p2,
                           double w, double h, int zX, int zY)
{
    
    //make sure it is actually possible to calculate the zones
    //if any parameters are <= 0, this function will
    //loop infinately
    if( zX <= 0 || zY <=0 || w <= 0 || h <= 0 )
        return NULL;
    vector<int>* zones = new vector<int>;
    
    //the width of each zone
    double xz = w/zX;
    double yz = h/zY;
    //clip the lines to the universe
    //find the stopping and starting directions
    int x,y;
    x = p1.x/xz;
    y = p1.y/yz;
    int stopX = p2.x /xz;
    int stopY = p2.y/yz;
    //get the direction
    Vector2 point = p1;
    Vector2 dir = ( p2- p1 ).u();
    zones->push_back( x+y*(zX+1) );
    while( stopX != x || stopY != y )
    {
        
       
        
        if( dir.x == 0 && dir.y == 0 )
            break;
        //now find the next zone
        if( dir.x == 0 )
        {
            if( dir.y > 0 )
                y++;
            else
                y--;
        }
        else
            if( dir.x > 0 )
            {
                double diffx = (x+1)*xz - point.x;
               // cout << point.x << ','<< point.y << ','<< diffx<< endl;
                double aY = point.y+diffx*dir.y/dir.x;
                double bY;
                if( dir.y == 0 )
                    x++;
                else if( dir.y > 0
                         &&  aY  >= ( bY = (y+1)*yz) )
                {
                    double oy=  point.y;
                    y++;
                    point.y = y*yz;
                    point.x += dir.x*(point.y-oy)/dir.y;
                    

                }
                else if( dir.y < 0
                         && aY < (bY=(y)*yz) )
                {
                    double oy=  point.y;
                    point.y = y*yz;
                    y--;
                   
                    point.x += dir.x*(point.y-oy)/dir.y;
                    
                }
                else
                {
                   
                    x++;
                    point.y +=  dir.y*(x*xz-point.x)/dir.x;
                    
                    point.x = x*xz;
                }

            }
        else
        {
            
            //dir.x < 0
            double diffx = (x)*xz - point.x;
            
            double aY = point.y+diffx*dir.y/dir.x;
            double bY;
            if( dir.y == 0 )
                x--;
            else if( dir.y > 0
                     &&  aY >= (bY =(y+1)*yz) )
            {
                double oy = point.y;
                y++;
                point.y = y*yz;
                point.x += dir.x*(point.y-oy)/dir.y;
                

            }
            else if( dir.y < 0
                     && aY < (bY = (y)*yz ) )
            {
                double oy = point.y;
                 point.y = y*yz;
                y--;
               
                point.x +=  dir.x*(point.y-oy)/dir.y;
                
            }
            else
            {
                
                double ox = point.x;
                point.x = x*xz;
                x--;
                point.y += dir.y*(point.x - ox)/dir.x;
                
            }
        }
        if( x > zX || y > zY || y < 0 || x < 0  )
        {

            return zones;
        }
        zones->push_back( x+y*(zX+1) );
    }


    return zones;

}

//*********************************************************************
Entity::~Entity(){}


//*********************************************************************
Vector2 Entity::Line::closest( Vector2 point, double * rangeOut )
{
    
    Vector2 l = (p2-p1);
    //first,  see if we are in the center areac
    double lmag2 = l.mag2();
	Vector2 toPointFromP1 = point-p1;
	double dot1 = l.dot(point-p1);
    if( l.dot(p2 - point) >= 0 && dot1 >= 0 && lmag2 > 0 )
    {
        
        
        Vector2 v = ( dot1/lmag2*l - toPointFromP1);
       

        
        if( rangeOut )
        {
           
            *rangeOut = v.mag();
        }
        return point+v;
    }
   
    double range = (p1 - point).mag2();
    Vector2 closest = p1;
    double  temp = (p2 - point).mag2();
    if( temp < range)
    {
        range = temp;
        closest = p2;
    }
    
    if( rangeOut )
        *rangeOut = sqrt( range );
    return closest;
};
//*********************************************************************

double  Entity::Line::rangeTo( const Vector2 &  start, const Vector2 & direction
                               , Vector2 * where)
{
	
    Vector2 arm1( p1 - start );
    Vector2 arm2( p1 - p2 );
    Vector2 perp( -arm2.y, arm2.x );
	perp= perp/perp.mag();
	double X = arm1.dot( perp );
    if( X * perp.dot( direction ) < 0 )
    {
        return -1;
    }
   
    double range = fabs(X/perp.dot(direction));
    Vector2 point = start+direction*range;
    if( where )
        *where=point;
    double a2mag2 = arm2.mag2() ;
	double dot = (point-p2).dot( arm2 );
    //now  to see if it is on the line
	if( dot < 0 || dot > a2mag2 )  
   // if( (point-p2).mag2() > a2mag2 ||(point-p1).mag2() > a2mag2    )
        return -1;
    else
        return range;
}