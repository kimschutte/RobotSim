/*
 *  Simulator2.cpp
 *    This file provides the functions of simulator that have to do with actually 
 *    Runing the simulation.
 *  Created by krs9249 on Fri Apr 11 2003.
 *  Copyright (c) 2003. All rights reserved.
 *  Log:
 *     $Log: Simulator2.cpp,v $
 *     Revision 1.2  2003/04/19 21:44:41  krs9249
 *     fixed getSensorValue calls.
 *
 */

#include "Wall.h"

#include "Strip.h"
#include <iostream.h>
#include "Simulator.h"
#ifdef MACOSX
#else
#define round rint
#endif

Vector2 Simulator::armCalc( const Vector2 & p, double theta, Vector2 in )
{
    double s = sin(theta);
    double c = cos(theta);
    return p + Vector2( in.x*c +-in.y*s, in.x*s + in.y*c ) ;
}
//**********************************************************************************
void Simulator::clearHistories()
{
    for( int i = 0; i < robots.size(); i++ )
    {
        robots[i]->historyClear();
    }
}
//**********************************************************************************
void Simulator::serviceSensor( Robot * r, Sensor& sens )
{
    double s = sin(r->loc.theta);
    double c = cos(r->loc.theta);
    Vector2 location( sens.location.x*c +-sens.location.y*s ,sens.location.x*s +sens.location.y*c ) ;
    location+=r->loc.point;

	//
	//  Goal sensor
	//
    if( sens.type == GoalSensor )
    {
        Vector2 toGoal = goal-r->loc.point;
        double angle = atan2( toGoal.y, toGoal.x );
        angle -= r->loc.theta;
        if( angle < 0 )
            angle += 2*M_PI;
        sens.fov = angle;
        r->updateSensor( &sens, time, toGoal.mag() );
    }
    else if( sens.type == Compass )
    {
         r->updateSensor( &sens, time, r->loc.theta );
    }
    else if( sens.type == TapeSensor )
    {

	//Tape Sensor
        double fStrip = 0;

	//First see if there is grid in this location
        if( strip > 0 )
        {
            double xdiff = location.x - strip*round( location.x/strip );
            double ydiff = location.y - strip*round( location.y/strip );
            if( fabs( xdiff ) < 0.5*swidth || fabs( ydiff ) < 0.5*swidth )
            {
                fStrip = 1;
            }
            else
            {
                
            }

        }
	//Then check for overlap on all the strips.
        vector< Entity* >::iterator itr2 =entities.begin() ;
        for( ; itr2 != entities.end(); itr2++ )
        {
            if( *itr2 == r || (*itr2)->getType() != Entity::kStrip)
                continue;
            double range;
            (*itr2)->getShape()->closest(location, &range );
            if( range < ((Strip *)(*itr2))->width*0.5 )
            {
                fStrip = 1;
            }

        }

        r->updateSensor( &sens, time, fStrip );
    }
    else if( sens.type == SonarFOV )
    {
        //first, find the zones
        //do this later

        double s1 = sin(r->loc.theta+sens.fov);
        double c1 = cos(r->loc.theta+sens.fov);
        double s2 = sin(r->loc.theta-sens.fov);
        double c2 = cos(r->loc.theta-sens.fov);
        double min = sens.range;
        Vector2 face ( sens.face.x*c +-sens.face.y*s, sens.face.x*s +sens.face.y*c ) ;
        Vector2 face1 ( sens.face.x*c1 +-sens.face.y*s1, sens.face.x*s1 +sens.face.y*c1 );
        Vector2 face2 ( sens.face.x*c2 +-sens.face.y*s2, sens.face.x*s2 +sens.face.y*c2 ) ;
        vector< Entity* >::iterator itr2 =entities.begin() ;
	
	
        
        for( ; itr2 != entities.end(); itr2++ )
        {
            //for each wall, clip it to the bounds of the cone, and find how close it is.
            //return the closest reading
           
            if( *itr2 == r || (*itr2)->getType() != Entity::kWall)
                continue;
            Vector2 where1, where2 ;
            Entity::Line *l = (Entity::Line *)(*itr2)->getShape();
            Vector2 a1 = l->p1-location;
            Vector2 a2 = l->p2-location;
            double range = 1000;
            double t1 = a1.dot(face)/a1.mag();
            double t2 = a2.dot(face)/a2.mag();
            if( t1 < 0 && t2 < 0 )
                continue;  //can't be in the area if it is behind it.
			double closestRange;
			(*itr2)->getShape()->closest( location,&closestRange );
            if( closestRange > sens.range )
				continue;
			
            double cosTheta = cos( sens.fov );
			
			double range1 = (*itr2)->getShape()->rangeTo(location, face1, &where1 );
            double range2 = (*itr2)->getShape()->rangeTo(location, face2, &where2 );
            
            
            if( range1 >=  0 )
            {
                if( range2 >= 0 )
                {
                    
                    Entity::Line( where2, where1 ).closest( location,&range );
                   
                }
                else if( t1 > cosTheta )
                {
                     Entity::Line( l->p1, where1 ).closest( location ,&range);
                    
                }
                else if( t2 > cosTheta )
                {
                    Entity::Line( l->p2, where1 ).closest( location,&range );
                }
                
                     
            }
            else if( range2 >=  0 )
            {
                //cout << "Here\n";
                if( t1 > cosTheta )
                {
                    Entity::Line( l->p1, where2 ).closest( location,&range );
                }
                else if( t2 > cosTheta )
                {
                    Entity::Line( l->p2, where2 ).closest( location,&range );
                }
            }
            else if( t1 > cosTheta )
            {
                range = closestRange;
            }
                
            if( range > 0 && range < min)
            {
                min = range;
            }

        }
        
        r->updateSensor( &sens, time, min );
    }
    else if( sens.type == Sonar )
    {
        //find sensor location
        //do a rotation
        //cout << "Sonar:" << sens.id << endl;
        Vector2 face ( sens.face.x*c +-sens.face.y*s, sens.face.x*s +sens.face.y*c ) ;

        //find the zones I need to check
        vector<int> *myZones = Entity::getLineZones( location , location + face* 100, w,h,zX,zY);

        //go through zone by zone compareing agains walls,
        //but only check the walls we havn;t already checked
        if( myZones == NULL  )
        {
            cout << "no zones returned\n";
            return;
        }
#ifdef BLOCK_REDUNDANT_LINE
        list< Entity* > checked;
#endif
        bool cont = true;
        double min = sens.range; //return maximum range if no reading
        int size = myZones->size() ;
        //cout<< "here\n";
        for( int  i = 0; i < size && cont; i++ )
        {
            
            int z = myZones->operator [] (i);
            //cout << z << endl; 
            if( z < 0 || z >= (zX+1) * (zY+1) )
            {
                cout << "dodged bad zone \n";
                cout << z << ':' << z%(zX+1) << endl;
                continue;
            }
            //if( z % (zX+1) >=zX )
            //    cout << "here" << endl;
            list< Entity* >& e( zones[z] );
            list< Entity* >::iterator eitr,citr;
            eitr =e.begin() ;
            //cout << "Here\n";
            for( ; eitr != e.end(); eitr++ )
            {
                
                if( *eitr == r || (*eitr)->getType() != Entity::kWall)
                {
                   // cout << "rejecting non-wall\n";
                    continue;
                }
                //cout << *eitr << endl;
#ifdef BLOCK_REDUNDANT_LINE
                bool found = false;
                citr = checked.begin();
                for(; citr != checked.end() && (*citr) <= (*eitr); citr++)
                {
                    if( *citr == *eitr )
                    {
                        found = true;
                        break;
                    }
                }
                if( found  )
                    continue;
#endif
                Vector2 loc;
                double range = (*eitr)->getShape()->rangeTo(location, face , &loc);
                if( range >= 0 && range < min)
                {
                    min = range;
                    //if our intersection point is in our zone... no need to continue
                    if( int(loc.x * zX / h ) + int(loc.y * zY / w )* (zX+1) == z )
                        cont = false;

                }
#ifdef BLOCK_REDUNDANT_LINE
                else
                {
                    //use an inserction sort
                    citr = checked.begin();
                    for(; citr != checked.end() && (*citr) < (*eitr); citr++)
                    {}
                    checked.insert( citr, *eitr );
                }
#endif

            }
        }


        /*
         vector< Entity* >::iterator itr2 =entities.begin() ;

         for( ; itr2 != entities.end(); itr2++ )
         {
             if( *itr2 == r || (*itr2)->getType() == Entity::kStrip)
                 continue;
             double range = (*itr2)->getShape()->rangeTo(location, face );
             if( range > 0 && range < min)
             {
                 min = range;
             }

         }*/
        //cout << min << endl;
        if( min < 1e10 )
        {
            r->updateSensor( &sens, time, min );

        }
        else
        {
            r->updateSensor( &sens, time, 100 );
        }
        delete( myZones );
    }
}

///\The big mama method :)
//**********************************************************************************
void Simulator::run()
{
    double newTime = (float(clock()))/CLOCKS_PER_SEC;
    if( newTime > lastRealTime + 1 )
    {

        realtime = (time-lastTime)/(newTime-lastRealTime);
        lastTime = time;
        lastRealTime = newTime;
        updated = true;
        
    }

    vector< Robot* >::iterator itr1;
    done = true;

    
    //1) First We run the AI and update the sensors
    itr1= robots.begin();
    for( ; itr1 != robots.end(); itr1++ )
    {
        //update sensors
        map< int, Sensor>::iterator sitr = (*itr1)->sensors.begin();
        for( ; sitr != (*itr1)->sensors.end(); sitr++ )
        {

            serviceSensor( *itr1, sitr->second );
        }
        //give each robot a chance to think.
#ifdef DEBUGROBOT

        (*itr1)->print();
#endif
        if( !((*itr1)->AI->process( time )) )
            done = false; // check for termination
    }

    //2) Then each mobile has its forces calculated
    itr1= robots.begin();
    for( ; itr1 != robots.end(); itr1++ )
    {
        Robot *r((*itr1) );
        Location2t loc = (*itr1)->loc;
        if( loc.point.x < 0 )
            r-> applyForce( Force( loc.point, Vector2( 100,0 )));
        if( loc.point.x > w )
            r-> applyForce( Force( loc.point, Vector2( -100,0 )));
        if( loc.point.y < 0 )
            r-> applyForce( Force( loc.point, Vector2( 0,100 )));
        if( loc.point.y >h )
            r-> applyForce( Force( loc.point, Vector2( 0,-100 )));

        //add a little fluid friction
        //using f=-b(v)^2 b = 0.1
        Vector2 friction = -0.1*sqrt(loc.momentum.dot(loc.momentum))*loc.momentum/r->mass/r->mass;
        if( friction.mag()  > 100 )
            cout << "F:" << friction.mag() << endl;
        Force test(loc.point,friction);
        r-> applyForce( test);
        //also some rotational friction
        double rtorque = -1*fabs(loc.angularMomentum)*loc.angularMomentum/(*itr1)->moment/(*itr1)->moment;
        r-> applyForce( Force( loc.point+Vector2(0.5,0), Vector2( 0, rtorque)));
        r-> applyForce( Force( loc.point+Vector2(-0.5,0), Vector2( 0, -rtorque)));


        //now check for collistions
        vector< Entity* >::iterator itr2 =entities.begin() ;

        for( ; itr2 != entities.end(); itr2++ )
        {
            if( (*itr2)->getType() == Entity::kWall && *itr2 != r )
            {
                double range;
                Vector2 where = (*itr2)->getShape()->closest( loc.point, &range );
                if( range < r->radius )
                {
                    //force = k * radius/range
                    //cout  <<"Range:"<< range << endl;
                    double force =  1000*(r->radius-range) / range ;
                    //cout  << "Force:"<< force << endl;
                    Vector2 direction = loc.point - where;
                    //cout << where.y-loc.point.y << endl;

                    direction = direction/direction.mag();
                    //if( direction.y !=  0 )
                    //   cout << "Dy" <<direction.y << endl;
                    //cout << "Direction:" << direction.x << ',' << direction.y << endl;
                    r-> applyForce( Force( where, force*direction) );

                }
            }

        }

    }

    //Now update the robots physical position
    itr1= robots.begin();
    for( ; itr1 != robots.end(); itr1++ )
    {
        //cout << "here\n";
        (*itr1)->updateTo( step, time );
        //(*itr1)->print();
    }

    

    time += step;

    
}
