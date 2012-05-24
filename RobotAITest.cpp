/*
 *  RobotAITest.cpp
 *  RobotSim
 *
 *  Created by krs9249 on Sun Mar 30 2003.
 *  Copyright (c) 2003 __MyCompanyName__. All rights reserved.
 *
 */

#include "Robot.h"
#include<iostream.h>
static double lastTime = 0;
static double lastLeft=0;
static double lastRight = 0;
static double rotate;
static double tryTurn = 0;
void RobotAI::initialize( Robot * robby )
{
    rob = robby;
    Sensor sonar={ SonarFOV,2,0,0,0,0,Vector2(-1,0),Vector2(1,0),M_PI/6,10 };
    rob->addSensor( &sonar );
    Sensor sonar2={ 
Sonar,3,0,0,0,0,Vector2(-0.2,0.8),Vector2(1.0/sqrt(2),1.0/sqrt(2)), 0, 10 };
    rob->addSensor( &sonar2 );
    Sensor sonar3={ 
Sonar,4,0,0,0,0,Vector2(-0.2,-0.8),Vector2(1.0/sqrt(2),1.0/-sqrt(2)), 0, 10 };
    rob->addSensor( &sonar3 );
    Sensor sonar5={ TapeSensor,5,0,0,0,0,Vector2(0.5,0.5),Vector2(0,-1) };
    rob->addSensor( &sonar5 );
    Sensor sonar6={ TapeSensor,6,0,0,0,0,Vector2(0.5,-0.5),Vector2(0,-1) };
    rob->addSensor( &sonar6 );
    Sensor sonar7={ TapeSensor,7,0,0,0,0,Vector2(0,0),Vector2(0,-1) };
    rob->addSensor( &sonar7 );
    //Sensor sonar8={ SonarFOV,8,0,0,0,0,Vector2(0,0),Vector2(1,0),M_PI/6,50 };
    //rob->addSensor( &sonar8 );
    lastLeft = rob->getSensorValue(0);
    lastRight = rob->getSensorValue(1);
}



int servo( Robot * rob, double speed, double turn , double time )
{

    double dur = time-lastTime;
    if( dur == 0 )
        return 0;
    double diffLeft = rob->getSensorValue(0) - lastLeft;
    if( diffLeft > M_PI )
        diffLeft-= 2*M_PI;
    else if( diffLeft < -M_PI )
        diffLeft+= 2*M_PI;
    double diffRight = rob->getSensorValue(1) - lastRight;
    if( diffRight > M_PI )
        diffRight-= 2*M_PI;
    else if( diffRight < -M_PI )
        diffRight+= 2*M_PI;

    //actual speed is the average of the speed of the two wheels
    double aspeed = rob->wheelRadius*( diffRight + diffLeft )/dur*0.5;
    
    //cout <<  rob->getSensorValue(0)<< ':' <<  rob->getSensorValue(1) << endl;
    double left;
    double right;
    left = right = speed-aspeed;

    double aturn = (diffRight - diffLeft )*rob->wheelRadius/rob->wheelBase/dur;
    double tForce = turn-aturn;
    left -= tForce;
    right += tForce;
    
    rob->setMotorCurrent( 0, left );
    rob->setMotorCurrent( 1, right );
    lastTime = time;
    lastRight = rob->getSensorValue(1);
    lastLeft = rob->getSensorValue(0);
}

// Param : time ( current time )
// Returns 1 - completion
//         0 - no completed
int RobotAI::process( double time )
{
    //cout << "proccessing\n";
    bool f,l,r;
    bool o =  rob->getSensorValue(2) < 5 || rob->getSensorValue(3) < 2 || rob->getSensorValue(4) < 2;
    f = rob->getSensorValue(7) > 0;
    l = rob->getSensorValue(5) > 0;
    r = rob->getSensorValue(6) > 0;
    double speed=0, turn=0;
    if( f  )
    {

        speed = 3;
        if( r )
            turn = -6;
        else if( l )
            turn = 6;
        
    }
    else
    {
        turn = tryTurn;
        if( r )
            turn = -1;
        else if ( l )
                turn = 1;
         speed = 2;
    }
    if( o )
    {
        if( rob->getSensorValue(2) < 3 )
            speed = -0;

        if(  rob->getSensorValue(4) < 3 && rob->getSensorValue(3) > rob->getSensorValue(4) )
            turn = +6;
        else turn = -6;
    }
    servo( rob, speed, turn, time );
    if( r )
        tryTurn = -3;
    if( l )
        tryTurn = 3;

    return 0;
}


