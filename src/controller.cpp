/**
 * Copyright (C) 2020 AUTH-ARL
*/

#include <controller.h>
#include <iomanip>
#include <ros/ros.h>
#include <ros/package.h>
#include <time.h>

using namespace std;
using namespace arma;


PerfConstraintsLWR::PerfConstraintsLWR(std::shared_ptr<arl::robot::Robot> robot)
: arl::robot::Controller(robot, "Performance Constraints")
{
	nh_ = ros::NodeHandle("~");
	this->robot = robot;
	time = 0.0;
	
	u = arma::zeros<arma::vec>(7);
	u_x = arma::zeros<arma::vec>(7);
	u_n = arma::zeros<arma::vec>(7);
	q = arma::zeros<arma::vec>(7);
	q_ref = arma::zeros<arma::vec>(7);
	

	pose = arma::zeros<arma::mat>(3, 4);
	pose_vec = arma::zeros<arma::vec>(7); //[position' quaternion']'
	pose_ref = arma::zeros<arma::mat>(3, 4);
	J = arma::zeros<arma::mat>(6, 7);
	Jsym = arma::zeros<arma::mat>(6, 7);
	
	p = arma::zeros<arma::vec>(3);
    p_ref = arma::zeros<arma::vec>(3);
    
    K_imp = arma::zeros<arma::mat>(6, 6);
    D_imp = arma::zeros<arma::mat>(6, 6);
    R= arma::zeros<arma::mat>(3, 3);
    Qprev = arma::zeros<arma::vec>(4);
    Quat_ref = arma::zeros<arma::vec>(4);

	// get the parameters
    readParameters();

    /*Initialize performance constraints
	* Arguments:
	* Performance indices [1, 2, 3]  1: Manipulability Index, 2: Minimum Singular Value, 3: Inverse Condition Number
	* Calculation methods: _serial, _parallel, _parallel_nonblock (not advised)
	* Gradient with respect to: [_cartesian (default), _joints] Selectable for only gradient calculation
	* Separate position and orientation calculation
	*/
    pConstraints.reset(new PC(_manipulability,  _serial, _joints, false));
    pConstraints->setVerbose(1); //Set debug info. Comment or set to 0 to disable
}

void PerfConstraintsLWR::readParameters()
{
    std::cout << "[readParameters] Reading Parameter list ..." << std::endl;
    // load parameters and gains
    nh_.getParam("stiff_transl", stiff_transl);
    nh_.getParam("stiff_rot", stiff_rot);
    nh_.getParam("damp_transl", damp_transl);
    nh_.getParam("damp_rot", damp_rot);

 //    if (nh_.hasParam("filename")) {
 //    	nh_.getParam("filename", filename);
 //    }

	// string DIR = ros::package::getPath("performance_constraints");
	// string PATH = DIR + "/experiments/" + filename + ".dat";

 //    //check if previous file exists and rename with _prev suffix
 //    ifstream f(PATH);
	// if (f.good()) {
	// 	string NEW_PATH = DIR + "/experiments/" + filename + "_prev.dat";
	// 	rename(PATH.c_str(), NEW_PATH.c_str());
	// }
	// f.close();

	// outStream.open(PATH);
	// if (outStream.is_open()) {
	// 	write2file = true;
	// 	cout << "Writing to: " << PATH << endl;
	// }
	// else
	// 	cout << "Error writing to: " << PATH << endl;

	
	// Print messages
    std::cout << "[readParameters] Parameters are successfully loaded. " << std::endl;

}

void PerfConstraintsLWR::setImpedanceParams() {
	//Impedance parameters
	arma::vec temp_k, temp_c;
	temp_k << stiff_transl << stiff_transl << stiff_transl << stiff_rot << stiff_rot << stiff_rot; //stiffness [WARNING! TEMPORARY MODIFICATION IN XY STIFFNESS]
	K_imp = diagmat(temp_k);
	temp_c << damp_transl << damp_transl << damp_transl << damp_rot << damp_rot << damp_rot; //damping
	D_imp = diagmat(temp_c);

}

void PerfConstraintsLWR::measure()
{
	// std::cout <<"start measure()" << std::endl;
	// read pose from robot

	pose = robot->getTaskPose().matrix().toArma().submat(0,0,2,3);

	xdotRaw = robot->getTwist().toArma(); //read Cartesian velocity (J*q_dot)
	
	//filter xdot
	double tau = 0.01;
	for (unsigned int i=0; i<6; i++) {
		xdot.at(i)=(xdotRaw.at(i)*robot->cycle + xdot.at(i)*tau) / (tau+robot->cycle);
	}

	// update current position & orientation
	p = pose.col(3);
	R = pose.submat(0, 0, 2, 2);
	// task_orientation_transform = arl::math::get6x6Rotation(R);

	//update current orientation
	Q = arl::math::rotToQuat(R);
	arma::vec te=(Q.t()*Qprev); if (te(0)<0.0) Q=-Q; //avoid discontinuity
	Qprev = Q; //save for next loop

	pose_vec.subvec(0,2) = p;
	pose_vec.subvec(3,6) = Q;

	//read Jacobian
    J = robot->getJacobian().toArma();
    q = robot->getJointPosition().toArma(); //read joint position
	qdot = robot->getJointVelocity().toArma(); //read joint velocity


}



void PerfConstraintsLWR::update()
{
	time += robot->cycle;

	// update orientation error
	// arma::vec te=(Q.t()*Quat_ref); if (te(0)<0.0) Quat_ref=-Quat_ref; //avoid incontinuity
	quatDiff = arl::math::getQuatDifference(Quat_ref, Q);
	e_o = 2.0*quatDiff.rows(1, 3);

	//Cartesian impedance control without inertia reshaping
	u_x = J.submat(0, 0, 2, 6).t() * ( K_imp.submat(0,0,2,2)*(p_ref - p) + D_imp.submat(0,0,2,2)*(/*v_ref*/ - xdot.subvec(0,2))  ) 
		+ J.submat(3, 0, 5, 6).t() * ( K_imp.submat(3,3,5,5)*e_o + D_imp.submat(3,3,5,5)*(/*omega_ref*/ - xdot.subvec(3,5)) ); 

	//Null space controller
	pConstraints->updateCurrentConfiguration(Q); //measure the robot's configuration and put it here
	pConstraints->get_Jsym_spatial(Q, Jsym); //calculate J from current q
	pConstraints->updateCurrentJacobian(Jsym); 
	pConstraints->calculateGradient();
	arma::vec A = pConstraints->getGradient();

	// u_n = ... ;

	//The controller
	u = u_x + u_n;

}

void PerfConstraintsLWR::command()
{
	robot->setJointTorque(u);
}

/**
 * @brief printForLogging, prints logging variables
 */
void PerfConstraintsLWR::print_to_monitor()
{
	if (fmod(time, 1.) < 0.001) { //plot every once in a while

		cout << "T:" << std::fixed << std::setprecision(1) << time; // << " [" << std::fixed << std::setprecision(1) <<robot->cycle*1000. << "ms]";

		cout << endl; //empty line
	}
}

void PerfConstraintsLWR::write_to_file(){
	if (write2file) {// && fmod(time, 0.01) < 0.001) { //enters when a 'filename' param is given in config.yml AND every few loops (smaller filesize)
		outStream << std::fixed << std::setprecision(6) << time << ","; //column 1

		for (int i=0; i<3; i++)
			outStream << std::fixed << std::setprecision(6) << p(i)<< ","; //2:4


		for (int i=0; i<4; i++)
			outStream << std::fixed << std::setprecision(6) << Q.at(i) << ","; //32:35 Quaternion orientation


		outStream << endl; //newline
	}
}


bool PerfConstraintsLWR::run()
{
	
	//Generate trajectory to follow
    pose = robot->getTaskPose().matrix().toArma().submat(0,0,2,3);
	p = pose.col(3);
	p_ref = p; //mirror current position
	pose_ref = pose;

	Qd = arl::math::rotToQuat(pose.submat(0, 0, 2, 2)); //[keep Q desired on the initial one]

	Quat_ref = Qd; //enable this in the case of pure orientation control
	
	
	// Qprev = Qd;

	//Save starting configuration
    q_ref = robot->getJointPosition().toArma();

    setImpedanceParams();

	//Switch to impedance control
	robot->setMode(arl::robot::Mode::TORQUE_CONTROL);

	// a mutex in robot should be locked to ensure no other controller is running
	// on this robot
	while (robot->isOk())
	{
		measure();
		update();
		command();
		print_to_monitor();
		write_to_file();
		robot->waitNextCycle();
		ros::spinOnce();
	}

	//close file if opened
	if (write2file) {
		cout << "closing file..." << endl;
		outStream.close();
	}

	cout << "Press [ENTER] to shutdown controller..." <<endl;

	//stop when finished
	robot->stop();
	cout << "Robot stopped\n";

	return true;
}

