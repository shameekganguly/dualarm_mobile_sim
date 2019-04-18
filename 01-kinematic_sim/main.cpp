#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Primitives.h"
#include "Sai2Simulation.h"
#include "timer/LoopTimer.h"
#include <unistd.h>

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;

/*************MODULE CONSTANTS*************/

#define Pi 3.14159

const string world_file = "resources/world.urdf";
const string robot_file = "resources/panda_arm_hand.urdf";
const string robot1_name = "PANDA1";
const string robot2_name = "PANDA2";
const string camera_name = "camera_fixed";
const string ee_link_name = "link7";


// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;



//Joint limits
vector<double> panda_joint_limits_max = {
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973
    
};
vector<double> panda_joint_limits_min = {
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973    
};




/*************FUNCTION PROTOTYPES*****************/

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

//function that randomly samples from joint space
double random_sample(double jointLimitMax, double jointLimitMin);


Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az);


/*************************************************/



int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
    Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

    // load robots

     //--------Load the world frames------/

       //define the transformation parameters for the robot frames
    double x1 = 0.0; double y1 = -0.5; double z1 = 1.0;
    double ax1 = 2.0; double ay1 = 0.0; double az1 = 1.0;

    double x2 = 0.0; double y2 = 0.0; double z2 = 1.0;
    double ax2 = -2.0; double ay2 = 0.0; double az2 = -1.0;

    Eigen::Affine3d r1 = create_rotation_matrix(ax1, ay1, az1);
  	Eigen::Affine3d t1(Eigen::Translation3d(Eigen::Vector3d(x1,y1,z1)));
  	Eigen::Affine3d T_world_robot1 = (t1 * r1);
  	Eigen::Matrix4d m1 = T_world_robot1.matrix();

  	Eigen::Affine3d r2 = create_rotation_matrix(ax2, ay2, az2);
  	Eigen::Affine3d t2(Eigen::Translation3d(Eigen::Vector3d(x2,y2,z2)));
  	Eigen::Affine3d T_world_robot2 = (t2 * r2);
  	Eigen::Matrix4d m2 = T_world_robot2.matrix();

  	//load the robots

    auto robot1 = new Sai2Model::Sai2Model(robot_file, false, T_world_robot1);
    int dof = robot1->dof();
    robot1->_q.setZero();
    robot1->_dq.setZero();

    auto robot2 = new Sai2Model::Sai2Model(robot_file, false, T_world_robot2);
    robot2->_q.setZero();
    robot2->_dq.setZero(); 



	/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "01-kinematic_sim", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
    glfwSetMouseButtonCallback(window, mouseClick);

    // cache variables
    double last_cursorx, last_cursory;



    // while window is open:
    while (!glfwWindowShouldClose(window))
	{
        // update robot position
        
        unsigned int K = dof-2;
        for(unsigned int i = 0; i<K;i++)
        {   

            robot1->_q[i] = random_sample(panda_joint_limits_max[i] ,panda_joint_limits_min[i]);
            robot2->_q[i] = random_sample(panda_joint_limits_max[i] ,panda_joint_limits_min[i]);
        }
        
        robot1->updateKinematics();
        robot2->updateKinematics();
        
		Eigen::Vector3d P1,P2;
        robot1->positionInWorld(P1,ee_link_name);
        robot2->positionInWorld(P2,ee_link_name);

        

        if ( (P1-P2).norm() > 0.3 ) //if bigger than one metre
        {	
        	printf("NOT FEASIBLE\n");
        	continue; //do not consider this, restart the search
        }
		
		cout << "FEASIBLE" << "\n \r" << P1.transpose() << "\n \r" << P2.transpose() << "\n \r" << (P1-P2).norm() << endl;

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot1_name, robot1);
        graphics->updateGraphics(robot2_name, robot2);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents();

        // move scene camera as required
        // graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
        Eigen::Vector3d cam_depth_axis;
        cam_depth_axis = camera_lookat - camera_pos;
        cam_depth_axis.normalize();
        Eigen::Vector3d cam_up_axis;
        // cam_up_axis = camera_vertical;
        // cam_up_axis.normalize();
        cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
        Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
        cam_roll_axis.normalize();
        Eigen::Vector3d cam_lookat_axis = camera_lookat;
        cam_lookat_axis.normalize();
        if (fTransXp) {
            camera_pos = camera_pos + 0.05*cam_roll_axis;
            camera_lookat = camera_lookat + 0.05*cam_roll_axis;
        }
        if (fTransXn) {
            camera_pos = camera_pos - 0.05*cam_roll_axis;
            camera_lookat = camera_lookat - 0.05*cam_roll_axis;
        }
        if (fTransYp) {
            // camera_pos = camera_pos + 0.05*cam_lookat_axis;
            camera_pos = camera_pos + 0.05*cam_up_axis;
            camera_lookat = camera_lookat + 0.05*cam_up_axis;
        }
        if (fTransYn) {
            // camera_pos = camera_pos - 0.05*cam_lookat_axis;
            camera_pos = camera_pos - 0.05*cam_up_axis;
            camera_lookat = camera_lookat - 0.05*cam_up_axis;
        }
        if (fTransZp) {
            camera_pos = camera_pos + 0.1*cam_depth_axis;
            camera_lookat = camera_lookat + 0.1*cam_depth_axis;
        }       
        if (fTransZn) {
            camera_pos = camera_pos - 0.1*cam_depth_axis;
            camera_lookat = camera_lookat - 0.1*cam_depth_axis;
        }
        if (fRotPanTilt) {
            // get current cursor position
            double cursorx, cursory;
            glfwGetCursorPos(window, &cursorx, &cursory);
            //TODO: might need to re-scale from screen units to physical units
            double compass = 0.006*(cursorx - last_cursorx);
            double azimuth = 0.006*(cursory - last_cursory);
            double radius = (camera_pos - camera_lookat).norm();
            Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
            camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
            Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
            camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
        }
        graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
        glfwGetCursorPos(window, &last_cursorx, &last_cursory);
        //end camera movement code

        usleep(300000);

	}

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
        case GLFW_KEY_ESCAPE:
            // exit application
            glfwSetWindowShouldClose(window,GL_TRUE);
            break;
        case GLFW_KEY_RIGHT:
            fTransXp = set;
            break;
        case GLFW_KEY_LEFT:
            fTransXn = set;
            break;
        case GLFW_KEY_UP:
            fTransYp = set;
            break;
        case GLFW_KEY_DOWN:
            fTransYn = set;
            break;
        case GLFW_KEY_A:
            fTransZp = set;
            break;
        case GLFW_KEY_Z:
            fTransZn = set;
            break;
        default:
            break;
    }
}

double random_sample(double jointLimitMax, double jointLimitMin)
{	
	 
	double r2 = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(jointLimitMax-jointLimitMin)));
	double jointInputVal = jointLimitMin + r2;
	return jointInputVal;	
	
}


void mouseClick(GLFWwindow* window, int button, int action, int mods) {
    bool set = (action != GLFW_RELEASE);
    //TODO: mouse interaction with robot
    switch (button) {
        // left click pans and tilts
        case GLFW_MOUSE_BUTTON_LEFT:
            fRotPanTilt = set;
            // NOTE: the code below is recommended but doesn't work well
            // if (fRotPanTilt) {
            //  // lock cursor
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            // } else {
            //  glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            // }
            break;
        // if right click: don't handle. this is for menu selection
        case GLFW_MOUSE_BUTTON_RIGHT:
            //TODO: menu
            break;
        // if middle click: don't handle. doesn't work well on laptops
        case GLFW_MOUSE_BUTTON_MIDDLE:
            break;
        default:
            break;
    }
}


/*
returns the transformation matrix given fixed angle representation of rotation
*/
Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

