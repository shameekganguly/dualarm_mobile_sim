#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

#define Pi 3.14159

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_hand.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Vector3d x;//quantity to store current task space position
	Vector3d xdot;//quantiy to store current task space velocity


	//Quantities for gravity compensation and coriolis/cfugal compensation
	Eigen::VectorXd g(dof); // Empty Gravity Vector
	Eigen::VectorXd b(dof); // coriolis and cfugal vector


	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);
	robot->position(x,link_name,pos_in_link); //position of end effector
	robot->linearVelocity(xdot,link_name,pos_in_link); //velocity of end effec

	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	//set q_desired
	VectorXd q_desired(7);
	q_desired << initial_q(0),initial_q(1),initial_q(2),
	initial_q(3),initial_q(4),initial_q(5),0.1;

	//set x_desired
	VectorXd x_desired(3);
	x_desired << 0.3,0.1,0.5;

	//set x_desired_traj
	VectorXd x_desired_traj_init(3);
	x_desired_traj_init << 0.3,0.1,0.5;
	VectorXd x_desired_traj(3);
	x_desired_traj = x_desired_traj_init; //initialize the vector

	double dt = 0.001; // 1/freq

	//open file to store csv
	std::ofstream traj_file;
	traj_file.open("/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw2/data4iv.csv");

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5

		robot->gravityVector(g); //update gravity vector
		robot->coriolisForce(b); // update coriolis/cfugal vector
		robot->Jv(Jv, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->nullspaceMatrix(N, Jv);
		robot->position(x,link_name,pos_in_link); //position of end effector
		robot->linearVelocity(xdot,link_name,pos_in_link); //velocity of end effec


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{	
			MatrixXd Kp(7,7); Kp.setZero(); MatrixXd Kv(7,7); Kv.setZero();
			Kp(0,0) = 400; Kp(1,1) = 400; Kp(2,2) = 400; Kp(3,3) = 400; Kp(4,4) = 400; Kp(5,5) = 400; Kp(6,6) = 50;
			Kv(0,0) = 50; Kv(1,1) = 50; Kv(2,2) = 50; Kv(3,3) = 50; Kv(4,4) = 50; Kv(5,5) = 50; 
			Kv(6,6) = -0.3196519; // tune this to get oscillatory motion

			command_torques = -Kp*(robot->_q - q_desired) - Kv*robot->_dq + b + g;
			//command_torques.setZero();  
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 200;
			double kv = 28.3 - 4.0;

			MatrixXd Kv(7,7); Kv.setZero();
			Kv(0,0) = 50; Kv(1,1) = 50; Kv(2,2) = 50; Kv(3,3) = 50; Kv(4,4) = 50; Kv(5,5) = 50; 
			Kv(6,6) = 50; // tune this to a good value
			Kv = 0.36*Kv; //hand tuning

			command_torques = Jv.transpose()*(Lambda*(kp*(x_desired-x) - kv*xdot)) + g; 
			//command_torques += -Kv*robot->_dq; // for part2c
			command_torques += N.transpose()*robot->_M*(-Kv*robot->_dq);
			//command_torques.setZero();
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 200;
			double kv = 28.3 - 4.0;

			MatrixXd Kv(7,7); Kv.setZero();
			Kv(0,0) = 50; Kv(1,1) = 50; Kv(2,2) = 50; Kv(3,3) = 50; Kv(4,4) = 50; Kv(5,5) = 50; 
			Kv(6,6) = 50; // tune this to a good value
			Kv = 0.36*Kv; //hand tuning

			VectorXd p =  J_bar.transpose()*g;

			//cout << p.transpose()  << endl;
			
			command_torques = Jv.transpose()*(Lambda*(kp*(x_desired-x) - kv*xdot) + p); 
			command_torques += -N.transpose()*robot->_M*(Kv*robot->_dq);
			//command_torques.setZero();
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{		
			//compute the desired trajectory 
			VectorXd delta_traj(3);
			delta_traj(0) = 0.1*sin(Pi*controller_counter*dt);
			delta_traj(1) = 0.1*cos(Pi*controller_counter*dt);
			delta_traj(2) = 0;
			x_desired_traj = x_desired_traj_init + delta_traj;

			VectorXd xdot_traj(3);
			xdot_traj(0) = 0.1*Pi*cos(Pi*controller_counter*dt);
			xdot_traj(1) = 0.1*Pi*(-sin(Pi*controller_counter*dt));
			xdot_traj(2) = 0;


			//controller
			double kp = 200;
			double kv = 28.3 - 4.0;

			MatrixXd Kv(7,7); Kv.setZero();
			Kv(0,0) = 50; Kv(1,1) = 50; Kv(2,2) = 50; Kv(3,3) = 50; Kv(4,4) = 50; Kv(5,5) = 50; 
			Kv(6,6) = 50; // tune this to a good value
			Kv = 0.36*Kv; //hand tuning

			//for part 4iii
			MatrixXd Kp(7,7); Kp.setZero();
			Kp(0,0) = 400; Kp(1,1) = 400; Kp(2,2) = 400; Kp(3,3) = 400; Kp(4,4) = 400; Kp(5,5) = 400; Kp(6,6) = 50; 

			VectorXd p =  J_bar.transpose()*g;

			//Lambda = MatrixXd::Identity(3,3); //for part 4ii

			command_torques = Jv.transpose()*(Lambda*(kp*(x_desired_traj-x) - kv*(xdot - xdot_traj) ) + p); 
			
			command_torques += -N.transpose()*( robot->_M*(  Kv*robot->_dq + Kp*robot->_q ) + b + g ); //add Kp term and b for part 4iii
			
		}

		traj_file << x(0) << "," << x(1) << "," << x(2) << "," 
		<< robot->_q(0) << "," << robot->_q(1) << "," << robot->_q(2) << "," << robot->_q(3) 
		<< "," << robot->_q(4) << "," << robot->_q(5) << "," << robot->_q(6) << "," << 
		x_desired_traj(0) << "," << x_desired_traj(1) << "," << x_desired_traj(2) <<   endl;
		
		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	traj_file.close();

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
