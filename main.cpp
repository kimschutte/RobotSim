#include <iostream.h>
#include <sstream>
#include "Simulator.h"
#ifdef MACOSX
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
ifstream maze ("maze");
Simulator sim( maze );
int mainWindow = -2;
int controlWindow = -1;
extern "C" void display();
extern "C" void displayControls();
extern "C" void controlClickFunc(int button, int state,
                                  int x, int y);
extern "C" void idle();
int main (int argc,  char * argv[]) {
    // insert code here...
    char buf [90];
    cout << getcwd(  buf, 89 ) << endl;
    
    glutInit( &argc, argv);
    glutInitDisplayMode( GLUT_DOUBLE );   
	glutInitWindowPosition(0,0);
    mainWindow = glutCreateWindow( "Output" );
    
    glutDisplayFunc(display);
    //glutIdleFunc(idle);
	glutInitWindowPosition(320,0);
    controlWindow = glutCreateWindow( "Control" );
    glutDisplayFunc(displayControls);
    glutMouseFunc(controlClickFunc);
    glutMainLoop();
    return 0;
}
void display()
{
    glClear( GL_COLOR_BUFFER_BIT );
    sim.draw();
    glFlush();
    glutSwapBuffers();
}
bool running = false;
void idle()
{
    static double last = 0;
    if( !sim.isDone() )
    {
        sim.run();
		sim.run();
        sim.run();
        sim.run();
        sim.run();
        sim.run();
        sim.run();
		sim.run();
        sim.run();
        sim.run();
        sim.run();
        sim.run();
		double newTime = (float(clock()))/CLOCKS_PER_SEC;
        if(  newTime - last > 0.0166 )
        {
            //cout << "\ntime:" << sim.getTime() << endl;
            last = newTime;
            glutSetWindow(mainWindow);
            glutPostRedisplay();
			if( sim.checkStatistics() )
			{
				glutSetWindow(controlWindow);
				glutPostRedisplay();
			}
        }
        
    }
    else
    {
        glutIdleFunc(NULL);
        running = false;
        glutSetWindow(controlWindow);
        glutPostRedisplay();
    }

    
}
void showString( double x,double y, const char * in )
{
    glPushMatrix();
    glTranslatef( x,y,0);
    glScalef( 0.0007,0.0007,1 );
    
    while( *in )
    {
        glutStrokeCharacter(GLUT_STROKE_ROMAN,*(in++));
    }
    glPopMatrix();
}


void displayControls()
{
    glClear( GL_COLOR_BUFFER_BIT );
    glColor3f( 1,1,1);
    glBegin( GL_LINE_LOOP );
    glVertex2f( -1,1 );
    glVertex2f( -0.5,1 );
    glVertex2f( -0.5,0.5 );
    glVertex2f( -1,0.5 );
    glEnd();
    
    if( running )
    {
        glBegin( GL_LINES );
        glVertex2f( -0.95,0.95 );
        glVertex2f( -0.95,0.55 );
        glVertex2f( -0.55,0.95 );
        glVertex2f( -0.55,0.55 );
        glEnd();
    }
    else
    {
        glBegin( GL_LINE_LOOP );
        glVertex2f( -0.95,0.95 );
        glVertex2f( -0.55,0.75 );
        glVertex2f( -0.95,0.55 );
        glEnd();
    }
    glBegin( GL_LINE_LOOP );
    glVertex2f( -0.5,1 );
    glVertex2f( 0,1 );
    glVertex2f( 0,0.5 );
    glVertex2f( -0.5,0.5 );
    glEnd();
    stringstream s;
    s  << "TimeRatio:"<< sim.getTimeRatio();
    showString( -1,-1, s.str().c_str() );
    showString( -0.4,0.8, "Clear" );
	showString( -0.4,0.6, "Path" );
	
	glBegin( GL_LINE_LOOP );
    glVertex2f( 0.5,0.5 );
    glVertex2f( 1,0.5 );
    glVertex2f( 1,0 );
    glVertex2f( 0.5,0 );
    glEnd();
	showString( 0.6,0.2, "Quit" );
	if( sim.sensorsShown() )
		glColor3f( 0,1,0 );//green;
	else
		glColor3f( 1,0,0 );//red
	glBegin( GL_LINE_LOOP );
    glVertex2f( 0,1 );
    glVertex2f( 0.5,1 );
    glVertex2f( 0.5,0.5 );
    glVertex2f( 0,0.5 );
    glEnd();
	
	showString( 0.1,0.7, "Sensors" );
	if( sim.historyShown() )
		glColor3f( 0,1,0 );//green;
	else
		glColor3f( 1,0,0 );//red
	glBegin( GL_LINE_LOOP );
    glVertex2f( 0.5,1 );
    glVertex2f( 1,1 );
    glVertex2f( 1,0.5 );
    glVertex2f( 0.5,0.5 );
    glEnd();
	showString( 0.6,0.7, "Path" );
    glFlush();
    glutSwapBuffers();
}


void controlClickFunc(int button, int state,
                                 int x, int y)
{
    float fx = (float(x))/glutGet(GLUT_WINDOW_WIDTH);
    float fy = (float(y))/glutGet(GLUT_WINDOW_HEIGHT);
    if( state == GLUT_DOWN )
        cout << "Click("  <<  fx << ',' << fy <<")\n";
    else
        return;
	int myButton = fx / 0.25;
    if( fy <= 0.25 )
    {
        //we are in the top button row
        
        switch( myButton )
        {
            case 0:
                running = !running;
                cout << "Pause/Play Button\n";
                if( running )
                    glutIdleFunc(idle);
                else
                    glutIdleFunc(NULL);
                break;
            case 1:
                
                cout << "Clear Button\n";
                sim.clearHistories();
                break;
			case 2:
				cout << "Show Sensors Button\n";
				sim.showSensors( !sim.sensorsShown() );
				break;
			case 3:
				cout << "Show Path Button\n";
				sim.showHistory( !sim.historyShown() );
				break;

        }
        
    }
	else if( fy <= 0.5 )
	{
		if( myButton == 3 )
			exit(0);
	}
    glutPostRedisplay();
}
