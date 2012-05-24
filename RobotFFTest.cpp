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

    //use one feild of view sentor in the front, and 9 line sensors around the robot
    Sensor sonar={ SonarFOV,2,0,0,0,0,Vector2(0.5,0),Vector2(1,0),M_PI/8,10 };
    rob->addSensor( &sonar );
    for( int i = 1; i < 10 ; i ++ )
    {
        double angle = i * M_PI/5;
        double s = sin( angle );
        double c = cos( angle );
        Sensor sonari={
            SonarFOV,2+i,0,0,0,0,Vector2(0.5 *c,0.5*s),Vector2(c,s), M_PI/8, 10 };
        rob->addSensor( &sonari );
    }
    Sensor goal ={ GoalSensor,12,0,0,0,0,Vector2(0.5,0),Vector2(1,0) };
    rob->addSensor( &goal );
    
    
    
    lastLeft = rob->getSensorValue(0);
    lastRight = rob->getSensorValue(1);
}


Vector2 feelForce( Robot* rob)
{
    Vector2 total(0,0);
    const double MAX_FORCE = 5;
    double rangeToGoal = rob->getSensorValue(12);
    double goalForce;
    //first, a goal seeking force with a fall off at > 10,
    // and a balistic rgion closer than that
    if( rangeToGoal > 10 )
    {
        goalForce = 20*MAX_FORCE/(rangeToGoal+10);
    }
    else 
    {
        goalForce = MAX_FORCE;//( rangeToGoal/10 );
    }
    double angleToGoal = rob->getSensor(12)->fov;
    total += goalForce * Vector2( cos( angleToGoal), sin(angleToGoal ) );
    

    //now add a repulsion for each sonar input
    //starting if we detect a wall withing 2 meters
    // walls will repel if closer than 2, and weekly attact othersize
    for( int i = 0; i < 10 ; i ++ )
    {
        double angle = i * M_PI/5;
        double value =  rob->getSensorValue(2+i);
        
        if( value < 3 )
        {
            total += -0.4 * (MAX_FORCE * (3-value )*0.5 * Vector2( cos( angle), sin(angle ) ));
            total += 0.4 * MAX_FORCE  * Vector2(  -sin(angle ),cos( angle) );
            //also add a rotational force about the obstacle
            
        }
        else if( value < 5 )
        {
            //total +=  0.1*(MAX_FORCE  * 0.5 * ( 5 - value ) * Vector2( cos( angle), sin(angle ) ));
            total += 0.4*MAX_FORCE  * Vector2(  -sin(angle ),cos( angle) ); 
        }
    }
    return total;
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
    
    //ut << diffLeft<< ':' << diffRight << endl;
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
   // cout << "proccessing\n";
    Vector2 force = feelForce( rob );
   // cout << "got forces\n";
    if( force.x < 0 )
    {
        force.x = 0;
        if( force.y > 0 )
            force.y += 5;
        else
            force.y += -5;
    }
    if( force.x > 3 )
    {
        force.x = 10;
    }
        servo( rob, force.x, force.y, time );

  
       return rob->getSensorValue(12)  < 1;
}


