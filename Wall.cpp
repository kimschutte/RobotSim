
#include "Wall.h"
vector<int>* Wall::getZones( double w, double h, int zX, int zY)
{
    if( zones != 0 ) 
	return zones;
    else
    {
        zones = getLineZones( myShape.p1,  myShape.p2, w,h,zX,zY );
        return zones;
    }
    
}
