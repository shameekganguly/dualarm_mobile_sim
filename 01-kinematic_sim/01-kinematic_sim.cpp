/*
main.cpp

Code for evaluating the kinematics of Bimanual Manipulation

Author: Varun Nayak


*/

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
#include <fstream>

using namespace std;
using namespace Eigen;

/*************MODULE CONSTANTS*************/

#define ENABLE_GRAPHICS_UPDATE


#define Pi 3.14159
#define DEG2RAD 0.01745329

#define OBJECT_SIZE 0.3	//size to be manipulated (with some tolerance)
#define OBJECT_SIZE_TOL 0.05	//object size tolerance
#define ONE_MILLIS 1000
#define SLEEP_TIME 0*ONE_MILLIS
#define SAMPLES_TO_AV 30000	//number of samples to compute mean position of workspace 30000 is the best
#define SAMPLES_TO_LIM 2*SAMPLES_TO_AV	//number of samples to find the maximum distances of workspace


const string world_file = "resources/world.urdf";
const string robot_file = "resources/panda_arm_hand.urdf";
const string robot1_name = "PANDA1";
const string robot2_name = "PANDA2";
const string camera_name = "camera_fixed";
const string ee_link_name = "leftfinger";


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
    2.8973,    1.7628,    2.8973,    -0.0698,    2.8973,    3.7525,    2.8973
    
};
vector<double> panda_joint_limits_min = {
    -2.8973,    -1.7628,    -2.8973,    -3.0718,    -2.8973,    -0.0175,    -2.8973    
};

//steps of x to optimize over (in degrees) 60-120, default 90
vector<double> x_steps = {
    60
};

//y is from 0-60, default 30
vector<double> y_steps = {
    30
};

//z is from 50-110, default 60
vector<double> z_steps = {
  110
};

//step of distance between the arms in metres 0.35-0.75, default 0.55
vector<double> d_steps = {
0.40
    };


const string object_name = "BOX";

/*************FUNCTION PROTOTYPES*****************/

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

void moveCamera(Eigen::Vector3d& camera_pos,Eigen::Vector3d& camera_lookat, Eigen::Vector3d& camera_vertical,Eigen::Vector3d& cam_up_axis, GLFWwindow* window);


//function that randomly samples from joint space
double randomSample(double jointLimitMax, double jointLimitMin);

//creates rotation matrix from fixed angle representation
Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az);

//samples from joint space of robot and updates the _q values
void robotJointSpaceSample(Sai2Model::Sai2Model* robot); 

void robotJointSpaceCounterUpdate(Sai2Model::Sai2Model* robot);


/*************************************************/



int main() {
	//cout << "Loading URDF world model file: " << world_file << endl;	//verbose

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
    Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
    graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	Eigen::Vector3d cam_up_axis;
	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this

    //open file to store csv
    std::ofstream data_file;
    data_file.open("/home/varun/sai2/apps/dualarm_mobile_sim/01-kinematic_sim/bimanual_data_v1.csv");
    bool simulation_in_process = true;

    //-------------------GRAPHICS---------------------//

	 // set up error callback
    glfwSetErrorCallback(glfwError);
    // initialize GLFW
    glfwInit();
    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);
    // information about computer screen and GLUT display window
	int screenW = mode->width;    int screenH = mode->height;    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;    int windowPosY = -(screenH - windowH) ;    int windowPosX = -2*windowPosY;
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




    // load robots

     //--------Load the world frames------/

    

    //-----------------BEGIN SIMULATION LOOP----------------//

    int number_of_iterations_complete = 0;

    // while window is open:
    while (!glfwWindowShouldClose(window) && simulation_in_process )
	{
       
      
       for(int i1 = 0; i1 < x_steps.size();i1++)
       {
       for(int i2 = 0; i2 < y_steps.size();i2++)
       {
       for(int i3 = 0; i3 < z_steps.size();i3++)
       {
       for(int i4 = 0; i4 < d_steps.size();i4++)
       {

        number_of_iterations_complete++;

        bool iteration_done =  false;

         //define the transformation parameters for the robot frames    
        //------PARAMETERS--------//
        double dist_between_arms = d_steps[i4]; //distance between the arms in mts
        double roll_angle = x_steps[i1]*DEG2RAD; //x-axis out of the plane
        double pitch_angle = y_steps[i2]*DEG2RAD;    // y-axis towards the right 
        double yaw_angle = z_steps[i3]*DEG2RAD; //z-axis upwards

        //parameters to rotation matrix
        double x1 = 0.0; double y1 = -dist_between_arms/2; double z1 = 1.0;
        double ax1 = roll_angle; double ay1 = pitch_angle; double az1 = yaw_angle;
        double x2 = 0.0; double y2 = dist_between_arms/2; double z2 = 1.0;
        double ax2 = -roll_angle; double ay2 = pitch_angle; double az2 = -yaw_angle;
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



         double total_size = x_steps.size()*d_steps.size()*y_steps.size()*z_steps.size();

        cout << "-----------PERCENTAGE COMPLETE: " << ((double)100*(number_of_iterations_complete -1))/total_size << "%" <<  " ------------  "<< endl;



        unsigned long no_of_samples = 0;
        double x_min = 0; double x_max = 0; double x_mean= 0;
        double y_min = 0; double y_max = 0; double y_mean= 0;
        double z_min = 0; double z_max = 0; double z_mean= 0;
        
        while(!iteration_done)
        {

               


                // update joint position of robot by random sampling
                robotJointSpaceSample(robot1);
                robotJointSpaceSample(robot2);
              	

                //FOR VISUALIZING ONLY
                //robot1->_q.setZero();
                //robot2->_q.setZero();
                //FOR VISUALIZING ONLY

              	//update kinematics in the model
                robot1->updateKinematics();
                robot2->updateKinematics();
                
                //create global position vectors of the end effectors
        		Eigen::Vector3d P1,P2;
                robot1->positionInWorld(P1,ee_link_name);
                robot2->positionInWorld(P2,ee_link_name);

                if (!( ((P1-P2).norm() < OBJECT_SIZE + OBJECT_SIZE_TOL) && ((P1-P2).norm() > OBJECT_SIZE - OBJECT_SIZE_TOL ) )) //if bigger than a certain distance (object size limit)
                {
                	continue; //do not consider this object size, resample
                }
                

                //compute the position of the box with respect to the end effectors
                Eigen::Vector3d object_position = (P1+P2)/2;
                Eigen::Quaterniond object_orientation = Quaterniond::Identity();

                no_of_samples++; //	successful sample

                //cout << "Object position " << object_position.transpose() << endl;

                if(no_of_samples<=SAMPLES_TO_AV)
                {
                	//calculate the mean
                	x_mean += - x_mean/no_of_samples + object_position[0]/no_of_samples;
                	y_mean += - y_mean/no_of_samples + object_position[1]/no_of_samples;
                	z_mean += - z_mean/no_of_samples + object_position[2]/no_of_samples;
                }
                else if(no_of_samples<=SAMPLES_TO_LIM)	//update  extreme points (limits of box)
                {	
                	//cout<< "MEAN = " << x_mean << "\t" << y_mean << "\t" << z_mean << endl;
                	//break;

                	if(object_position[0]-x_mean < x_min - x_mean)
                	{
                		x_min = object_position[0];
                	}
                	if(object_position[1]-y_mean < y_min - y_mean)
                	{
                		y_min = object_position[1];
                	}
                	if(object_position[2]-z_mean < z_min - z_mean)
                	{
                		z_min = object_position[2];
                	}


                	if(object_position[0]-x_mean > x_max - x_mean)
                	{
                		x_max = object_position[0];
                	}
                	if(object_position[1]-y_mean > y_max - y_mean)
                	{
                		y_max = object_position[1];
                	}
                	if(object_position[2]-z_mean > z_max - z_mean)
                	{
                		z_max = object_position[2];
                	}

                }
                else
                {
                	double dim_x_box = abs(x_max-x_min);
                	double dim_y_box = abs(y_max-y_min);
         	       	double dim_z_box = abs(z_max-z_min);
         	       	double workspace_box_volume = dim_z_box*dim_y_box*dim_x_box;

                    data_file << roll_angle << "," << pitch_angle << "," << yaw_angle << "," << dist_between_arms << ","
                                << workspace_box_volume << endl;

                    iteration_done = true;

                   

                }




                //----------GRAPHICS-----------//
                #ifdef ENABLE_GRAPHICS_UPDATE
        			// update graphics. this automatically waits for the correct amount of time
        			int width, height;
        			glfwGetFramebufferSize(window, &width, &height);
        	        graphics->updateGraphics(robot1_name, robot1);
        	        graphics->updateGraphics(robot2_name, robot2);
        	        graphics->updateObjectGraphics(object_name, object_position, object_orientation);
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
        		    //move camera according to cursor and keys
        		    moveCamera(camera_pos,camera_lookat, camera_vertical, cam_up_axis, window);
        		    //update camera in graphs
        	        graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
         		
        		#endif

         		//sleep in seconds		
                usleep(SLEEP_TIME);
        } // iteration loop


       }
       }   
       }
       }//closing the for loop search

        simulation_in_process = false;
	}

    data_file.close();
    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}






//-----------------------------------------------------------------------------//


//--------------------------FUNCTIONS----------------------------------------------------

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

double randomSample(double jointLimitMax, double jointLimitMin)
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

void robotJointSpaceSample(Sai2Model::Sai2Model* robot)
{

	 unsigned int K = robot->dof()-2;
    for(unsigned int i = 0; i<K;i++)
    {   

        robot->_q[i] = randomSample(panda_joint_limits_max[i] ,panda_joint_limits_min[i]);
    }

}


void robotJointSpaceCounterUpdate(Sai2Model::Sai2Model* robot)
{

	static unsigned int long counter = 0;
	unsigned int K = robot->dof()-2;
    for(unsigned int i = 0; i<K;i++)
    {  
        robot->_q[i] = (double)counter/100;
    }
    counter++;

}




void moveCamera(Eigen::Vector3d& camera_pos,Eigen::Vector3d& camera_lookat, Eigen::Vector3d& camera_vertical, Eigen::Vector3d& cam_up_axis, GLFWwindow* window)
{	
	    static double last_cursorx, last_cursory;

        // move scene camera as required
        // graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
        Eigen::Vector3d cam_depth_axis;
        cam_depth_axis = camera_lookat - camera_pos;
        cam_depth_axis.normalize();
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
  		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

}











