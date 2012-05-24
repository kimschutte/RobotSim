




#ifndef WALL_H
#define WALL_H
#include "Entity.h"
#include "Vector2.h"

class Wall: public Entity
{
public:
	Wall(){ initShape(Vector2(),Vector2()); type = kWall; };
	Wall( Vector2 p1i, Vector2 p2i ){ initShape(p1i,p2i); type = 
kWall; };
	Wall( double x1, double y1, double x2, double y2 )
	{ initShape( Vector2( x1,y1), Vector2(x2,y2) ); type = kWall; };
	virtual vector<int>* getZones( double w, double h, int zX, int zY);
	virtual Shape * getShape(){ return &myShape;};
private:
	void initShape( Vector2 p1, Vector2 p2 )
	    { zones = NULL; myShape.p1=p1; myShape.p2=p2;}; 
	//Vector2 p1,p2;
	Line myShape;
	vector<int>* zones;
};
#endif

