/*
 *  Simulator.cpp
 *  RobotSim
 *
 *  Created by krs9249 on Sun Mar 30 2003.
 *  Copyright (c) 2003 __MyCompanyName__. All rights reserved.
 *
 */

#include "Wall.h"
#include "Simulator.h"
#include "Strip.h"
#include <iostream.h>
#include <string>
#include <math.h>
#ifdef MACOSX
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#define round rint
#endif

#define xBLOCK_REDUNDANT_LINE
#define xDRAWZONES

Simulator::Simulator():
w(50),
h(50),
zX(10),
zY(10),
time(0),
step( 0.0001 ),
sensorsShow( 0 ),
pathShow( 0 ), 
nextCriticalTime ( -1 ),
done( false ),
strip( 0 ),
swidth( 0 )
{
    init();
    zones = new ZoneList[ (zX+1) * (zY+1)];
    addEntity( new Robot()); // add one robot for testing

}

Simulator::Simulator( ifstream& in ):
w(50),
h(50),
zX(10),
zY(10),
time(0),
step( 0.0001 ),
sensorsShow( 0 ),
pathShow( 0 ), 
nextCriticalTime ( -1 ),
done( false ),
strip( 0 ),
swidth( 0 )
{
    init();
    zones = new ZoneList[(zX+1) * (zY+1)];
    

    //load the wall from a file:
    string inString;
    while( in )
    {
        in >> inString;
        if( inString.compare( "wall:" ) == 0 )
        {
            double x1,y1,x2,y2;
            in >> x1 >> y1 >> x2 >> y2;
            Entity * wall =  new Wall( x1,y1,x2,y2 );
            addEntity( wall );
            
        }
        else
        if( inString.compare( "strips:" ) == 0 )
        {
           
            in >> strip >> swidth;
            

        }
        else
        if( inString.compare( "strip:" ) == 0 )
        {
            double x1,y1,x2,y2,w;
            in >> x1 >> y1 >> x2 >> y2 >> w;
            Entity * s =  new Strip( x1,y1,x2,y2 ,w);
            addEntity( s );


        }
        else
        if( inString.compare( "goal:" ) == 0 )
        {
            double x1,y1;
            in >> x1 >> y1;
            goal.x = x1;
            goal.y = y1;
        }
    }
    addEntity( new Robot()); // add one robot for testing
}

Simulator::~Simulator()
{

    vector< Entity* >::iterator itr1= entities.begin();
    for( ; itr1 != entities.end(); itr1++ )
    {
        if( *itr1 ) //shold never be null, but might as well be safe.
            delete *itr1;
        else
            clog<<"A null entity discovered...This is Bad...So yell at programmer\n";
    }
    delete [] zones;
}
void Simulator::init()
{
    lastTime = 0;
    lastRealTime = 0;
}
void Simulator::addEntity( Entity * in )
{
    if( !in )
    {
        cerr << "someone is trying to add a NULL entity!\n";
        return;
    }
    entities.push_back( in );
    
    vector<int> * ezones = in->getZones( w,h, zX, zY );
    if( ezones )
    {

        for( int i = 0; i < ezones->size() ; i++ )
        {
            int zone = ezones->operator[](i);
            if( zone >= 0 && zone < (zX+1) * (zY+1) )
                {
                    zones[zone].push_back( in );
                    //cout << zone<< endl;
                }
                
        }
    }
    //in the future sort for zone here
    // as this will vastly improve efficiency :)
    if( in->getType() == Entity::kRobot )
        robots.push_back( (Robot * ) in );
}

void Simulator::draw()
{
    //first, fill the window
   
    
    glLoadIdentity();
    glTranslatef( -0.8,-0.8,0 );
    glScalef( 2/w,2/h,1 );
    glScalef( 0.8,0.8,1 );
    //first draw the strips
    glColor3f( 0.2, 0.2, 0.6);
    if( strip > 0 )
    {
        for( double i = 0; i < h+swidth/2; i+= strip )
        {
            glBegin( GL_QUAD_STRIP );
            
            glVertex2f( w, i-swidth/2);
            glVertex2f( w, i+swidth/2);
            glVertex2f( 0, i-swidth/2);
            glVertex2f( 0, i+swidth/2);
            glEnd();
        }
        for( double i = 0; i < h+swidth/2; i+= strip )
        {
            glBegin( GL_QUAD_STRIP );
            glVertex2f( i+swidth/2, h );
            glVertex2f( i-swidth/2, h );
            glVertex2f( i+swidth/2, 0 );
            glVertex2f( i-swidth/2, 0 );
            
            
            
            glEnd();
        }

    }
    //now draw the robot and walls
    vector< Entity* >::iterator itr1= entities.begin();
    
    for( ; itr1 != entities.end(); itr1++ )
    {
        
        if( (*itr1)->getType() == Entity::kRobot )
        {
            
            Robot& rob( *((Robot*)*itr1) );
            double x,y;
            x = rob.loc.point.x;
            y = rob.loc.point.y;
            glColor3f( 1.0, 0, 0);
            
            glBegin( GL_TRIANGLE_FAN );
            glVertex2f( x, y );
            for( int i = 0; i <= 10 ; i++ )
            {
                glVertex2f( x+rob.radius*cos( i*M_PI/5), y+ rob.radius*sin( i*M_PI/5) );
            }
            glEnd();
            //white
            glColor3f( 1,1,1);
            glBegin( GL_LINES );
            glVertex2f( x, y );
            glVertex2f( x+2*rob.radius*cos( rob.loc.theta), y+2*rob.radius*sin( rob.loc.theta) );
            glEnd();
			if( sensorsShow )
            {
				map< int, Sensor>::iterator sitr = rob.sensors.begin();
				for( ; sitr != rob.sensors.end(); sitr++ )
				{
					drawSensor( &rob, sitr->second );
				}
			}
            //Draw the robots history
			if( pathShow )
			{
				glColor3f( 1,0,1);
				glBegin( GL_LINE_STRIP );
				list<Vector2>::iterator hitr = rob.history.begin();
				for(; hitr !=  rob.history.end(); hitr++ )
				{
					glVertex2f( hitr->x, hitr->y );
				}
				glEnd();
			}
        }
        else if( (*itr1)->getType() == Entity::kWall )
        {
            Wall& w( *((Wall*)*itr1) );
            Entity::Line  l(*(( Entity::Line *) w.getShape()));
#ifdef DRAWZONES
            vector<int> *zonesl  =w.getZones( this->w,h,zX,zY );
	    if( zonesl )
		{
		    double dX = this->w/zX;
		    double dY = h/zY;
		    glColor3f(0,1,0);
		    for( int i = 0; i < zonesl->size(); i++ )
		    {
			int z = zonesl->operator[] (i);
			int x = z % (zX+1);
			int y = z / (zX+1);
			glBegin( GL_LINE_LOOP );
			glVertex2f( x*dX, y*dY );
			glVertex2f( x*dX+dX, y*dY );
			glVertex2f( x*dX+dX,y*dY+dY );
			glVertex2f( x*dX, y*dY+dY );
			glEnd();
		    }
		}
#endif
            glColor3f( 1,1,1);
            glBegin( GL_LINES );
            glVertex2f( l.p1.x , l.p1.y );
            glVertex2f( l.p2.x , l.p2.y );
            glEnd();

        }
        else if( (*itr1)->getType() == Entity::kStrip )
        {
            Strip& s( *((Strip*)*itr1) );
            //find normal vector

            Vector2 n = ( s.p1-s.p2 ).s().u()*s.width/2;
            glColor3f( 0.2,0.2,0.6);
            glBegin( GL_POLYGON );
            glVertex2f( s.p1.x + n.x , s.p1.y + n.y );
            glVertex2f( s.p1.x - n.x , s.p1.y - n.y );
            glVertex2f( s.p2.x - n.x , s.p2.y - n.y );
            glVertex2f( s.p2.x + n.x , s.p2.y + n.y );
            glEnd();

        }
       
            
    }
    //draw the goal
    glColor3f( 0.2,0.2, 0.8 );
    glBegin( GL_LINES );
    glVertex2f( goal.x-0.5, goal.y - 0.5 );
    glVertex2f( goal.x+0.5, goal.y + 0.5 );
    glVertex2f( goal.x+0.5, goal.y - 0.5 );
    glVertex2f( goal.x-0.5, goal.y + 0.5 );
    glEnd();
}

void Simulator::drawSensor( Robot * r, const Sensor& sens )
{
    if( sens.type == ShaftEncoder )
        return;
    glPushMatrix();

    glTranslatef( r->loc.point.x , r->loc.point.y , 0 );
    glRotatef( r->loc.theta*180.0/M_PI, 0,0,1 );
    glTranslatef( sens.location.x, sens.location.y,0 );
    glRotatef( atan2( sens.face.y,sens.face.x )*180.0/M_PI, 0,0,1 );

    if( sens.type == Sonar )
    {
        glColor3f( 0,1,0);
        glBegin( GL_POLYGON );
        glVertex2f( -0.5, -0.5 );
        glVertex2f( 0, 0);
        glVertex2f( -0.5, +0.5 );
        glEnd();
        glBegin( GL_LINES );
        glVertex2f( 0, 0);
        glVertex2f(  sens.reading, 0 );
        glEnd();
    }else if( sens.type == SonarFOV )
    {
        glColor3f( 1,1,0);
        glBegin( GL_LINE_LOOP );
        glVertex2f( 0, 0);
       // cout << "FOV:"<<sens.fov << endl;
        for( float a = -sens.fov; a <=  sens.fov; a +=  sens.fov/10 )
        {
            //cout << a << endl;
            glVertex2f( sens.reading*cos(a), sens.reading*sin(a) );
        }
        
        glEnd();
        
    }else if ( sens.type == TapeSensor )
    {
        if( sens.reading == 0 )
            glColor3f( 0,0,0);
        else
            glColor3f( 0.5,0.5,0);
        glBegin( GL_POLYGON );
        glVertex2f( -0.1, -0.1 );
        glVertex2f( -0.1, 0.1);
        glVertex2f( 0.1, 0.1);
        glVertex2f( 0.1, -0.1 );
        glEnd();
    }
    glPopMatrix();
}

