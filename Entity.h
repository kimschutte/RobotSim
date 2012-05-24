/*
 *  Entity.h
 *  Description:
 *       An entity is anythign that can exist as an object within the 
 *       simulator.  The class provieded here has the accessor funcitons
 *       for performing the geometry neccessary for simulation.
 *
 *  Created by krs9249 on Sun Mar 30 2003.
 *  Copyright (c) 2003 . All rights reserved.
 *  Log:
 *     $Log: Entity.h,v $
 *     Revision 1.3  2003/04/18 21:38:59  krs9249
 *     added Documention.
 *
 */
#ifndef ENTITY_H
#define ENTITY_H
#include <list>
#include <vector>
#include "Vector2.h"
#include <iostream.h>
using std::list;
using std::vector;


class Entity
{
public: //static functions

    // Function: getLineZones
    // Description: This returns the zones along a line, starting at p1
    //              and traversing to p2;
    // Parameters:
    //      p1 : The first endpoint of the line
    //      p2 : the second endpoint of the line
    //      w : width of the universe
    //      h : hight of the universe
    //      zX: number of zones in the X direction
    //      zY: number of zones in the Y direction
    // Returns: a pointer to a vector, it is the callers responsibility to de-allocate
    //
    
    static vector<int>* getLineZones( Vector2 p1, Vector2 p2,
                                      double w, double h, int zX, int zY);

	// This method will be implemented when I figure out how
	// For FOV sensors check against everything.
	//
    static vector<int>* getSectorZones( Vector2 p1, Vector2 p2,
                                        double w, double h, int zX, int zY, double theta){return NULL;};
public:
    virtual ~Entity();

public: // member types

	//Enums for identifying identities and their shapes
    enum EType { kNothing,kRobot, kWall, kStrip};
    enum EShape { kLine,kCircle,kBox,kAgregate };

     //this shape subclass is here to prevent conflicts with other shape
     // classes.
    class Shape
    {
        public:
        EShape type;
        
       // virtual bool intersectShape( Shape & in );

	// Method: closest
        // Description:
	//     This method returns the closest point on the shape to 
	//     the caller.
	// Paremeters:
	//      point - the point to find the closest point to.
	//      rangeOut - if non -null, the range to the point returned from point.
	// Returns:
	//	The point on this shape closest to the point passed in.
	//
        virtual Vector2 closest( Vector2 point, double * rangeOut ){return Vector2();};

	//
	// Method: rangeTo
	// Description: 
	//     Given a point and a direction, this will find the intersection of the resulting ray
	//     and this shape.
	// Parameters:
	//	Start - the starting point of the ray.
	//	direction - a normalized direction vector indicating the direction of the
	//                  ray.
	//      where - if non - null used to return the point of intersection.
	// Returns:
	//      The distance from start to the point of intersection.
	//
        virtual double rangeTo( const Vector2 &  start, const Vector2 & direction,
                            Vector2 * where){ return -1;};
        double rangeTo( const Vector2 &  start, const Vector2 & direction ){ return  rangeTo( start, direction, NULL );};

    };

    class Line: public Shape
    {
        public:
        Vector2 p1,p2;
        Line( ){type =  kLine;};
        Line( const Vector2 & p1i, const Vector2 & p2i ):p1(p1i),p2(p2i){type =  kLine;};
        virtual Vector2 closest( Vector2 point, double * in );
        virtual double rangeTo( const Vector2 &  start, const Vector2 & direction,Vector2 * where );
    };

public: // accessors

	// Accessor: getType
	// 	Returns the type of entity this is. 
    EType getType(){ return type; };

	// Accessor: getZones
	//     Returns any zones which the enbtity overlaps, or null if 
	//     it is not supported by that entity.
    virtual vector<int>* getZones( double w, double h, int zX, int zY){ return NULL;} 


	// Accessor: getShape
	//     Returns the shape associated with this entity, if any.
	//
    virtual Shape * getShape(){ return NULL; };
    
   
	
     	
    
    
protected: EType type;
};

#endif
