/*
 *  Robot.cpp
 *  RobotSim
 *
 *  Created by krs9249 on Sat Mar 29 2003.
 *  Copyright (c) 2003 __MyCompanyName__. All rights reserved.
 *
 */


#include <iostream.h>

#include "Robot.h"
#include <math.h>
//Constructors:
Robot::Robot():
mass( 1 ),
moment( 1 ),
radius( 1 ),
wheelBase( 2 ),
wheelRadius( 1 ),
motorConstant( 2 ),
iLeft(0),iRight(0),
p(0),
w(0),
loc( 20,20,M_PI )

{
    type = kRobot;
    clearForce();
    Sensor wheelLeft = { ShaftEncoder, 0, 0 ,0,0,0,0,0,0};
    Sensor wheelRight = { ShaftEncoder, 1 ,0,0,0,0,0,0,0};
    sensors[0] = wheelLeft;
    sensors[1] = wheelRight;


    //use the defualt AI;
    AI = new RobotAI;
    AI->initialize( this );
}

Robot::~Robot()
{
    delete AI;
}


//Sensor manipualtion
void Robot::removeSensor( int ID )
{
    map<int,Sensor>::iterator itr = sensors.find( ID );
    if( itr != sensors.end() )
        sensors.erase( itr );
}

Sensor * Robot::getSensor( int ID )
{
    map<int,Sensor>::iterator itr = sensors.find( ID );
    if( itr != sensors.end() )
        return &(itr->second);
    else
        return &sensors[0];

}

void Robot::updateSensor( Sensor * s, double time, double value )
{
    //first see if the sensor is valid
    if( !s )
        return;

    if( s->reading != value )
    {
        s->reading = value;
        if( s->armed )
        {
            s->armed = 0;
            s->capturedTime = time;
        }
    }
}




//simulator interface:
void Robot::applyForce( Force in )
{
    //cout << "ForceInX:" << in.force.x << endl;
    //cout << "ForceInY:" << in.force.y << endl;
    //first we have to find the component not permendicular to the wheels
    //as they have perfect traction :)
    Vector2 facing( loc.theta );
    totalExternalForce += in.force.dot( facing ) * facing;
 

    //the torque is the cross product of the force and the line from the robots center.
    double rx = in.point.x-loc.point.x;
    double ry = in.point.y-loc.point.y;
    totalExternalTorque+= rx*in.force.y-ry*in.force.x;
    //cout << "Torque:" << totalExternalTorque << endl << endl;
    //cout << "Force:" << totalExternalForce.mag() << endl;
    

}

Vector2 Robot::totalForce()
{
    //first, finde the force due to each wheel
    Vector2 forceLeft,forceRight;
    double force = iLeft*motorConstant/wheelRadius;
    //then, we find the direction
    forceLeft.x = cos( loc.theta ) * force;
    forceLeft.y = sin( loc.theta ) * force;
    force = iRight*motorConstant/wheelRadius;
    forceRight.x = cos( loc.theta ) * force;
    forceRight.y = sin( loc.theta ) * force;
    //cout << "ext:" << totalExternalForce.x << endl;
    return forceLeft + forceRight + totalExternalForce;
}

double Robot::totalTorque()
{
    double force = iLeft*motorConstant/wheelRadius;
    double torqueLeft = force*wheelBase*0.5;
    force = iRight*motorConstant/wheelRadius;
    double torqueRight =force*wheelBase*0.5;
    return totalExternalTorque+torqueRight -torqueLeft;
}

Location2t Robot::getNextLocation( double duration)
{
    //first, find the new location using the good old
    // d(t) = start + vt + 1/2*a*t*t
    Vector2 velocity = loc.momentum/mass;
    Vector2 force = totalForce();
   // cout << "force:"<< force.x << endl;
    Vector2 accel = force/mass;
    Vector2 newLoc= loc.point + velocity * duration + 0.5*duration*duration*accel;

    //now find the new facing  with
    //theta(t) = start + wt + 1/2( w' t*t )
    double avelocity = loc.angularMomentum/moment;
    double torque = totalTorque();
    double aaccel = torque/moment;
    double newTheta= loc.theta + avelocity * duration + 0.5*duration*duration*aaccel;


    //Note: there is a small error due to the fact that the wheels are ideal,
    // but it is insignificant if the duration is small enough.
     //now the momentum is re-directed by the ideal wheels
    Vector2 newMomementum = loc.momentum+force*duration;
    Vector2 face( newTheta );
    newMomementum = newMomementum.dot( face )*face;
    //cout << "momentum:" <<  newMomementum.x << endl;
    return Location2t( newLoc,newMomementum , newTheta, loc.angularMomentum+torque*duration);
    
}

void  Robot::updateTo( double duration, double time )
{
    historyAdd(loc.point );
    Location2t newLoc = getNextLocation( duration );
    //now to update the wheels shaft encoders
    //because this is the small case model, we can do this linearly with
    //only insignificatn erros ( which increase with duration )
    Vector2 displacement = newLoc.point -loc.point;
    //find the component in line with the vehicles facing
    double roll = displacement.dot( Vector2( loc.theta ) ) / wheelRadius;
#ifdef DEBUGROBOT
    cout << displacement.x << endl;
    cout << Vector2( loc.theta ).x << endl;
    cout << roll << endl;
#endif
    //and find hte change in wheel position due to rotation
    double rotate = (newLoc.theta - loc.theta) * wheelBase/2 /wheelRadius;
    //Because I know these sensors exist.., I do not need to check return values
    Sensor * left = getSensor( 0 );
    Sensor * right = getSensor( 1 );
    double newLeft = left->reading + roll - rotate;  
    double newRight = right->reading + roll + rotate;
    //map newRight .left 0 < x < 2PI
    newLeft = newLeft - 2 * M_PI * floor( newLeft / ( 2 * M_PI ));
    newRight = newRight - 2 * M_PI * floor( newRight / ( 2 * M_PI ));
    updateSensor( left, time, newLeft );
    updateSensor( right, time, newRight );
    oldLoc = loc;
    loc = newLoc;
    loc.theta = loc.theta - 2 * M_PI * floor( loc.theta / ( 2 * M_PI ) );
    
    clearForce();
    
}

int Robot::print()
{
    cout << "Robot  location ( x,y, theta )\n";
    cout << '(' <<loc.point.x << ", " <<loc.point.y << ", " << loc.theta<< ")\n";
    cout << "Endoder ( left, right )\n";
    cout << '(' <<getSensorValue(0) << ", " <<getSensorValue(1) << ")\n";
    return 0;
}

void Robot::historyAdd( const Vector2 & in)
{
   
    bool first;
	first = history.empty();
    list< Vector2 >::iterator last;
    last = history.begin();
    history.push_front( in );
    if( first)
    {
        lastStep = history.begin();
        distcount = 0;
    }
    else
    {
        distcount += (in - *last).mag();
        //cout << distcount << endl;
    }
    if( distcount > 0.1 )
    {
        //cout << "Consolidating history\n";
        history.erase( ++history.begin() , lastStep );
        distcount = 0;
        lastStep = history.begin();
    }
    

    
}
