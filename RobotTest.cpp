/*
 *  RobotTest.cpp
 *  RobotSim
 *
 *  Created by krs9249 on Sun Mar 30 2003.
 *  Copyright (c) 2003 __MyCompanyName__. All rights reserved.
 *
 */

#include "Robot.h"
#include <iostream.h>

int main()
{
    Robot t;
    cout << floor( -0.5 ) << endl;;
    cout << "Robot Created\n";
    
    t.setMotorCurrent( 0, 1 );
    t.setMotorCurrent( 1, -1 );
    double time = 0;
    for(; time < 2.0; )
    {
        cout << "time:" << time << endl;
        t.print();
        cout <<  endl;
       
        time += 0.01;
        
        t.applyForce( 
        t.updateTo( 0.01, time );
    }
    
    
}

