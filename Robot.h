// File: $Id: Robot.h,v 1.5 2004/05/28 13:39:12 krs9249 Exp krs9249 $
// Description:
//	This file defines the robot class, as well as the interface
//  used by the AI to interace with the robot.  Also included are several 
//  several of the structures used by the robot class.
//
//  Author:  Kim Schuttenberg
//  
//  Log:
//	$Log: Robot.h,v $
//	Revision 1.5  2004/05/28 13:39:12  krs9249
//	some minor optimizations
//
//	Revision 1.4  2003/04/19 21:44:07  krs9249
//	fixed getSensorValue member name
//	/
//
//	Revision 1.3  2003/04/18 17:44:15  krs9249
//	Added much documentation
//
//


#ifndef ROBOT_H
#define ROBOT_H
#include <map>
#include "Vector2.h"
#include "Entity.h"



using std::map;


//
//This structure is used to represent a force applied to the robot
//
//
struct Force
{
    Vector2 point, force;
    Force():point(),force(){};
    Force( Vector2 p, Vector2 f ):point(p),force(f){};
};

//
// This structure provides all the information 
// needed to predicet the robots current and next positions.
//
struct Location2t{
    Vector2 point;
    Vector2 momentum;
    double theta;
    double angularMomentum;
    Location2t():angularMomentum(0){};

    Location2t( double x, double y, double theta_in ):
        point( x,y ),
        momentum(),
        theta( theta_in ),
        angularMomentum( 0 ){};


    Location2t( double x, double y, double px, double py, double theta_in, double am ):
        point( x,y ),
        momentum( px,py ),
        theta( theta_in ),
        angularMomentum( am ){};
    Location2t( Vector2 loc, Vector2 m, double theta_in, double am ):
        point( loc ),
        momentum(m),
        theta( theta_in ),
        angularMomentum( am ){};
};

// enumerations indicating sensor types
//
//
enum SensorType
{
    TapeSensor,   //a sensor for detecting the stip object
    Sonar,	  //a sensor for detecting walls, by line intersection
    ShaftEncoder, //a sensor for detecting wheel position
    SonarFOV,     //a range sensor with a programmable field of view
		  //note: computationally intesive to simulate.
    Compass,	  //a sensor for detecting current orientation
    GoalSensor    //a sensor which gives the range and angle to the goal.
};



struct Sensor
{
	SensorType type;
	int 	   id;
		// reserved ID's
		// 0 Shaft encoder left
		// 1 shaft encoder right
	double 	   reading;
		//The current reading for the sensor
		//if this is a goal sensor this is the distance
	double 	   capturedTime;
			//if armed, when the value is changed this is
			//set to the time of the change.
	int 	   armed;  //capture on change, sets to false on a change
	double 	   quantization;   //ignored for now
	Vector2	   location, face; //ignored for compass
        double     fov; //field of view for this sensor, if it needs one
			//for the goal sensor, this is the angle to the 
			//goal
        double     range;
			//the range which the sensor can sense over;
			//used for sonar type sensors.
    
};

//
// Class: Robot
//	This is the structure used by the simulator to represent a robot
//	Each robot is associated with a single "RobotAI" object.
//      

class RobotAI; //  a pre-declaration to make this work
class Robot : public Entity
{
public: //friends
    friend class Simulator;
    
public: //constructors.
    	// Default constructor
	// Atatches to a default AI object
    	Robot();
    	virtual ~Robot();
public://Robot configuration
       //Robot Mechanics
	
	// Method: setWheelBase
	// Description:
	//	This sets the robots wheels to be width away from each other.
	//      The wheels are set on oposite sides of the robots center,
	//      Pointed in the robots foward direction.
	//  Parameters:
	//      width	- The distance between the wheels
	//      radius  - The radius of the wheel.
    void setWheelBase( double width, double radius ){ wheelBase = width; wheelRadius = radius;};
    	
	// Method: setMotorConstant
 	// Description: This sets the relationship between current and 
	//              force for each motor.
	// Parameters:
	//		NpA - the number of newtons generate per amp
	void setMotorConstant( double NpA ){ motorConstant = NpA; };

	// Method: setMass
	// Description: This sets the mass of the robot
	// Paramter: kg - The mass in kilograms.
    void setMass( double kg ){ mass = kg; };

	// Method: setMomentOfInertia
	// Description: This sets the moment of inertia of the robot
	// Parameter: kgmm - the second moment of inirtia of the robot.
    void setMomentOfInertia( double kgmm ){ moment = kgmm; };

	// Method: setRadius
	// Description:  This sets the radius of the robot
	// Parameter:    m - the radius in meters.
    void setRadius( double m ){ radius = m;};

    //sensor configuration
    
    // method: addSensor
    // Description: This adds a sensor to the robots sensor list, by copy
    //              
    //  Parameter:
    //         s - pointer to the sensor to add.
    //             The sensor ID should be unique
    
    void addSensor( Sensor * s)
    {
        if( s )
            sensors[s->id] =*s;
    };


    //  
    // Method: removeSensor
    // Description:  This removes the named sensor from the robot
    //              
    // Parameter: 
    //               ID - the ID of the sensor to remove.
    //
    void removeSensor( int ID );

public: //Accessors

	// Method: getSensorValue
	// Description: This returns the reading field of the sensor specified
	//              In the future it will apply noise and 
	//              quantization errors 		
 	// Parameter: ID - the ID of the sensor to get the value for
        //            If this is not valid, bad things happen!
	//
	// Returns:
	//       the value of the reading field of the indicated sensor.
        double getSensorValue( int ID ){ return getSensor(ID)->reading; };

	// Method: getSensor
	// Description:  This returns the a pointer to the sensorSensor in the robots 
	//               list coresponding to the noted ID.
        // Parameters:	 
	//		ID - the id of the sensor to retreive
	// Returns:	
	//	The pointer to the named sensor or NULL
        //      Note: This is not valid after the sensor is removed or the robot destructed. 
	//
    Sensor * getSensor( int ID);

	// Method: getSensorTime
	// Description:
	//    This method works in conjuction with armSensor.
        //    It returns the time at which the sensor was last changed while
	//    It was armed.
	// Paremeter: 
	//	ID - the id of the sensor to read the time of
	// Returns:
	//      The time the sensor was last changed when armed.
    double getSensorTime( int ID ){ return getSensor(ID)->capturedTime;};

	// Method: armSensor
	// Description:
	//	This method sets the sensor status to armed, such that
	//	The next time the sensor changes value the time will be 
	//      Recoreded ( see getSensorTime above )
	//	Note: when the sensor is changed, the armed status resets
	//	to false.
	// Paremeter - ID - the ID of the sensor to arm.
    void armSensor( int ID ){ getSensor(ID)->armed = 1;};


	// Method: updateSensor
	// Description:
	//	This method will set the value of a sensor, and will 
	//      also set the time if the sensor is armed and the provided
	//      value is different from the last value.
	//
	// Paremeters:
	// 		s - the sensor to update
	//		time - the current time
	//              value - the new value of the sensor
    void updateSensor( Sensor * s, double time, double value );

public: //mutators

        // Name: Set Positions
        // Sets the robots X,Y coordinants and nullifies any momentum
	// Used incase the AI has to "cheat" to recover from an error
        void setPosition( double x, double y ){ loc = Location2t( x,y,0); };


public: //Motor control

	// Method: setMotorCurrent
	// Description: 
	//   This will set the current driving one of the 
	//   robots motors.
	//  Parementers
	//	ID - the ID of the motor to set the current for
	//		0 = left
	//		1 = right
	//	i - the current to apply to the motor.
    void setMotorCurrent( int ID, double i )
    {
        if( ID == 1)
            iRight = i;
        else
            iLeft = i;
    };

public:
        //simulator interface

	// Method: applyForce
	// Description: This method applies a force to the robot, adding appropriately
	//              To the total force and total torque for this round of simulation
	// Paremeter: in - the force to appy.
        void applyForce( Force in );

	// Method: totalForce
	// Desctiption: This accessor returns the total force recored so far for 
	//              this round of simulation.
	// Returns - the force oberseved so far for thsi round of simulaiton.
    Vector2 totalForce();

	// Method: totalTorque
	// Description: This accessor returns the total tourque recored so far 
	//              for this round of simualtion.
	// Returns - the total torque observed so far. 
    double totalTorque();

	// Method: clearForce
	// Description:  This sets teh toal force and torque to 0
    void clearForce(){ totalExternalForce = Vector2(0,0); totalExternalTorque =0; } ;

    	// Method: getNextLocation
	// Description:
	//    This method calculates the next location of the robot based on
	//    The momentum of the robot and its current position.
	// Paremeter:
	// 	duration - the time over which the robot will have moved.
	// Returns:
	//      the next location
    Location2t getNextLocation( double duration );

	// Method: updateTo
	// Description: This updates the robot to the next simulation step
	//		This updates the positions, momentums, and shaft encoders.
	// Paremeters:
	//	duration - the time till the next simualtion step.
	//	time - the current time.
	//
    void updateTo( double duration, double time );

public:
        //Debugging
    int print();
    //history update
    void historyClear(){history.clear(); };
    void historyAdd( const Vector2 & in);
public:
    //member variables
    double mass;
    double moment;
    double radius;
    double wheelBase;
    double wheelRadius;
    double motorConstant;
private:

	//The internal force accumulators.
    Vector2  totalExternalForce;
    double totalExternalTorque;

    //motor currents
    double iLeft,iRight;
                        
    double p;//translational momentum
    double w;//rotational momentum
    Location2t loc;      //The current 
    Location2t oldLoc;   //and previous locations
			 //could be used for increasing accuracy of 
			 //sensor trigger times in the future.
    
    map<int,Sensor> sensors; // The internal sensor map
    RobotAI * AI;            // The AI for the robot, currently created and 
                             // destroyed byt the robot
    
	// Variables used to generate the robots path history.
    list< Vector2 > history; 
    list< Vector2 >::iterator lastStep;
    double distcount;
};

class RobotAI
{


//
// Robot Generation
//
public:
virtual void initialize( Robot * robby );

// Param : time ( current time )
// Returns 1 - completion
//         0 - no completed
virtual int process( double time );


private:
Robot * rob;
};
#endif
