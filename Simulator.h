/*
 *  Simulator.h
 *  RobotSim
 *
 *  Created by krs9249 on Sun Mar 30 2003.
 *  Copyright (c) 2003. All rights reserved.
 * Log:
 *     $Log: Simulator.h,v $
 *     Revision 1.5  2004/05/28 13:39:12  krs9249
 *     some minor optimizations
 *
 *     Revision 1.4  2003/04/18 21:41:05  krs9249
 *     checked in correct version.
 *
 */
#ifndef SIMULATOR_H
#define SIMULATOR_H
#include <string>
#include <fstream>
#include "Robot.h"
#include <list>
#include <vector>
using namespace std;

//
// Simulator maintains a list of objects in zones to 
// speed up sonar search.
//
typedef list< Entity* > ZoneList;



class Simulator
{
public: //control varaibles
    
    
     
public: //constructors
    Simulator();

	//
	// This constructor reads in a file specifying
	// The objects in the world.
	//
    Simulator( ifstream& in );
    //note, this will delete all added entities
    ~Simulator();

	//
	// Method: run 
	// Description:
	//     This method will run the simulation for one timestep
	//
    void run();
    	//
	// Method: draw
	// Description:
	//      This method uses openGL to draw into the current context
	//	It wil draw all objects in the simulation.
	void draw();

	//
	// Accessor: getTime
	// Description:
	//     This method returns the current simualtion time
    double getTime( ){ return time;};

	//
	// Accessor: isDone
	// Description:
	//	This method returns weather all robots 
	//	Have reported mission complete.	
	//
    bool isDone( ){ return done; };


public: //accessors
        
	//
	// Method: getTimeRatio
	// Description:
	//     This method returms the simulation
	//     time: real time ratio, calculated every second
	//
        double getTimeRatio(){ return realtime ;}

    // Method:  checkStatistics
    // Description:
    //      This checks whether the statistics have been updated,
    //      and sets the updated status to false
    bool checkStatistics(){
        bool retval = updated;
        updated = 0;
        return retval; };
	
	
	//return the drawing state of the history and sensors
	bool historyShown(){return pathShow; };
	bool sensorsShown(){return sensorsShow ;};
	
public: //mutators

	//
	// Method: clearHistories
	// Description:
	//      This method erases the recorded paths of all the
	//      Robots
	//
        void clearHistories();
	
	//control weather or not these are drawn
		void showHistory( bool show ) { pathShow = show;};
		void showSensors( bool show ){ sensorsShow = show;};
private:
	//
	// These functions are called by other functions
	// and should generally not be called by outside functions.
	//
    void addEntity( Entity * in );
    void init();
    list< ZoneList*>* findZones( Entity * in ){ return NULL;};//not used
    ZoneList& zone( int x, int y ){return zones[x+y*zX];}; //not used
    void serviceSensor( Robot * r, Sensor& s );
    void drawSensor( Robot * r, const Sensor& sens );
    Vector2 armCalc( const Vector2 & p, double theta, Vector2 in );
private: //member variables
    
    double w,h; //size of the simulated aread
    int  zX,zY; //number of zones in X and Y direction
    ZoneList * zones; //entities sorted by zone of influence
    
    vector<Entity *> entities; //all entities in simulation
    vector<Robot *> robots;
    double time;
    double step;
    double nextCriticalTime;
    bool done;
	bool sensorsShow;
	bool pathShow;
    double strip,swidth;
    //The location of the goal
    Vector2 goal;

    //simulation speed measuring variables
    bool updated;
    double lastTime;
    double lastRealTime;
    double realtime; //how close to realtime we are
};

#endif

