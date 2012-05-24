/*
 *  Strip.h
 *  RobotSim
 *
 *  Created by krs9249 on Tue Apr 01 2003.
 *  Copyright (c) 2003 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef STRIP_H
#define STRIP_H
#include "Entity.h"
#include "Vector2.h"

class Strip: public Entity
{
public:
    Strip():p1(),p2(),width(0.5){  type = kStrip;initShape(); };
    Strip( Vector2 p1i, Vector2 p2i,double widthi ):p1(p1i),p2(p2i),width(widthi){type =
        kStrip;initShape(); };
    Strip( double x1, double y1, double x2, double y2 ,double widthi):
        p1(x1,y1),p2(x2,y2),width(widthi){ initShape();type = kStrip; };

    
    virtual Shape * getShape(){ return &myShape; };
public:
    Vector2 p1,p2;
    double width;
private:
       
    
    void initShape(){ myShape.p1=p1; myShape.p2=p2;};
    Line myShape;
};
#endif

