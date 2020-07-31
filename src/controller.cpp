/**
 * Copyright (C) 2020 AUTH-ARL
*/

#include <controller.h>
#include <iomanip>
#include <ros/ros.h>
#include <ros/package.h>
#include <time.h>
// #include <unistd.h> //used for custon getch()
// #include <termios.h> //used for custon getch()

using namespace std;
using namespace arma;


PerfConstraintsLWR::PerfConstraintsLWR(std::shared_ptr<arl::robot::Robot> robot,
	std::shared_ptr<arl::robot::Sensor> ft_sensor)
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
	qdot_ref = arma::zeros<arma::vec>(7);
	

	pose = arma::zeros<arma::mat>(3, 4);
	pose_vec = arma::zeros<arma::vec>(7); //[position' quaternion']'
	pose_ref = arma::zeros<arma::mat>(3, 4);
	J = arma::zeros<arma::mat>(6, 7);
	Jinv = arma::zeros<arma::mat>(7, 6);
	JinvW = arma::zeros<arma::mat>(7, 6);
	M = arma::zeros<arma::mat>(7, 7);
	Minv = arma::zeros<arma::mat>(7, 7);
	
	p = arma::zeros<arma::vec>(3);
    p_ref = arma::zeros<arma::vec>(3);
    
    K_d = arma::zeros<arma::mat>(6, 6);
    C_d = arma::zeros<arma::mat>(6, 6);
    M_d = arma::zeros<arma::mat>(6, 6);
    K_eq = arma::zeros<arma::vec>(6);
    C_extra = arma::zeros<arma::vec>(6);

    R= arma::zeros<arma::mat>(3, 3);
    Qprev = arma::zeros<arma::vec>(4);
    Quat_ref = arma::zeros<arma::vec>(4);

    v_ref=arma::zeros<arma::vec>(6);
    F_v=arma::zeros<arma::vec>(6);
    U_v=arma::zeros<arma::vec>(6);

    ft_sensor_ = ft_sensor;

	// get the parameters
    readParameters();

    /*Initialize performance constraints
	* Arguments:
	* Performance indices [1, 2, 3]  1: Manipulability Index, 2: Minimum Singular Value, 3: Inverse Condition Number
	* Calculation methods: _serial, _parallel, _parallel_nonblock (not advised)
	* Gradient with respect to: [_cartesian (default), _joints] Selectable for only gradient calculation
	* Separate position and orientation calculation
	*/
    // pConstraints.reset(new PC(_manipulability,  _serial, _joints, false));
     
    /*Initialize performance constraints
	* Arguments:
	* 1: w_cr for translation or combined
	* 2: w_th for translation or combined
	* 3: w_cr for rotation [OPTIONAL: Leave empty for combined indices]
	* 4: w_th for rotation [OPTIONAL: Leave empty for combined indices]
	* 5: lambda for translation or combined
	* 6: lambda for rotation  [OPTIONAL: Leave empty for combined indices]
	* 7: Performance indices [_manipulability, _MSV, _iCN]  
	* 8: Calculation methods: _serial, _parallel, _parallel_nonblock (not advised)
	* 9: Gradient with respect to: [_cartesian (default), _joints] Selectable for only gradient calculation
	*/
    w_thT=0.14;
    w_crT=0.03;
    w_thR=0.5;
    w_crR=0.1;
    lambda = 1.0;
    pConstraints.reset(new PC(w_crT, w_thT, w_crR, w_thR, lambda, lambda, _MSV, _serial)); //Using MSV (better for human robot interaction)
    // pConstraints->setVerbose(1); //Set debug info. Comment or set to 0 to disable
}

void PerfConstraintsLWR::readParameters()
{
    std::cout << "[readParameters] Reading Parameter list ..." << std::endl;
    // load parameters and gains
    nh_.getParam("use_impedance", use_impedance); 
    nh_.getParam("stiff_transl", stiff_transl);
    nh_.getParam("stiff_rot", stiff_rot);
    nh_.getParam("damp_transl", damp_transl);
    nh_.getParam("damp_rot", damp_rot);
    nh_.getParam("inertia_transl", inertia_transl);
    nh_.getParam("inertia_rot", inertia_rot);
    nh_.getParam("nullspace_gain", nullspace_gain);
    nh_.getParam("nullspace_damping", nullspace_damping);


    if (nh_.hasParam("filename")) {
    	nh_.getParam("filename", filename);
    }

	string DIR = ros::package::getPath("performance_constraints_lwr");
	string PATH = DIR + "/experiments/" + filename + ".dat";

    //check if previous file exists and rename with _prev suffix
    ifstream f(PATH);
	if (f.good()) {
		string NEW_PATH = DIR + "/experiments/" + filename + "_prev.dat";
		rename(PATH.c_str(), NEW_PATH.c_str());
	}
	f.close();

	outStream.open(PATH);
	if (outStream.is_open()) {
		write2file = true;
		cout << "Writing to: " << PATH << endl;
	}
	else
		cout << "Error writing to: " << PATH << endl;

	
	// Print messages
    std::cout << "[readParameters] Parameters are successfully loaded. " << std::endl;

}

void PerfConstraintsLWR::setImpedanceParams() {
	//Impedance parameters
	arma::vec temp_k, temp_c, temp_m;
	temp_k << stiff_transl << stiff_transl << stiff_transl << stiff_rot << stiff_rot << stiff_rot; //stiffness [WARNING! TEMPORARY MODIFICATION IN XY STIFFNESS]
	K_d = diagmat(temp_k);
	temp_c << damp_transl << damp_transl << damp_transl << damp_rot << damp_rot << damp_rot; //damping
	C_d = diagmat(temp_c);
	temp_m << inertia_transl << inertia_transl << inertia_transl << inertia_rot << inertia_rot << inertia_rot; //inertia
	M_d = diagmat(temp_m);

}

void PerfConstraintsLWR::measure()
{
	// std::cout <<"start measure()" << std::endl;
	// read pose from robot

	pose = robot->getTaskPose().matrix().toArma().submat(0,0,2,3);

	xdot = robot->getTwist().toArma(); //read Cartesian velocity (J*qdot_ref)
	
	//filter xdot [Something is wrong with this part and the robot stops]
	// double tau = 0.01;
	// for (unsigned int i=0; i<6; i++) {
	// 	xdot.at(i)=(xdotRaw.at(i)*robot->cycle + xdot.at(i)*tau) / (tau+robot->cycle);
	// }

	// update current position & orientation
	p = pose.col(3);
	R = pose.submat(0, 0, 2, 2);
	task_orientation_transform = arl::math::get6x6Rotation(R);

	//update current orientation
	Q = arl::math::rotToQuat(R);
	arma::vec te=(Q.t()*Qprev); if (te(0)<0.0) Q=-Q; //avoid discontinuity
	Qprev = Q; //save for next loop

	pose_vec.subvec(0,2) = p;
	pose_vec.subvec(3,6) = Q;

	//read Jacobian
    J = robot->getJacobian().toArma();
    Jinv = arma::pinv(J);

   	M = robot->getMassMatrix().toArma();

    q = robot->getJointPosition().toArma(); //read joint position
	qdot = robot->getJointVelocity().toArma(); //read joint velocity

	//read external wrench from ATI and transform to base frame
	ati_forces_tool = ft_sensor_->getData().toArma();
	ati_forces_ = task_orientation_transform * ati_forces_tool; //rotate to base frame

}



void PerfConstraintsLWR::update()
{
	time += robot->cycle;

	
	// pConstraints->calculateGradient(q); //calculate the gradient of a performance index [see constructor for details]
	// A = pConstraints->getGradient();

	if (use_impedance) { //send torques to implement Cartesian impedance
		// update orientation error
		// arma::vec te=(Q.t()*Quat_ref); if (te(0)<0.0) Quat_ref=-Quat_ref; //avoid incontinuity
		quatDiff = arl::math::getQuatDifference(Quat_ref, Q);
		e_o = 2.0*quatDiff.rows(1, 3);

		//Cartesian impedance control without inertia reshaping
		u_x = J.submat(0, 0, 2, 6).t() * ( K_d.submat(0,0,2,2)*(p_ref - p) + C_d.submat(0,0,2,2)*(/*v_ref*/ - xdot.subvec(0,2))  ) 
			+ J.submat(3, 0, 5, 6).t() * ( K_d.submat(3,3,5,5)*e_o + C_d.submat(3,3,5,5)*(/*omega_ref*/ - xdot.subvec(3,5)) ); 

		//Null space controller
		Minv = arma::inv(M);
		JinvW = Minv * J.t() * arma::inv( J * Minv * J.t() ); //dynamically consistent inverse 

		u_n = ( arma::eye<arma::mat>(7,7) - J.t() * JinvW.t() ) * ( nullspace_gain * A - nullspace_damping * qdot ); 

		//The controller
		u = u_x + u_n;
	}
	else { //Certesian admittance control 
		pConstraints->updatePC(q); //Performance constraints are calculated in here
		F_v = pConstraints->getSingularityTreatmentForce();
		A = pConstraints->getGradient();

		if (pConstraints->getPerformanceIndex(0) <= w_thT) //the position part of w is considered here
			K_eq.subvec(0,2) = lambda * arma::pow(A.subvec(0,2), 2) / pow(pConstraints->getPerformanceIndex(0) - w_crT, 2);
		else
			K_eq.subvec(0,2).zeros();

		if (pConstraints->getPerformanceIndex(1) <= w_thR) //the orientation part of w is considered here
			K_eq.subvec(3,5) = lambda * arma::pow(A.subvec(3,5), 2) / pow(pConstraints->getPerformanceIndex(1) - w_crR, 2);
		else
			K_eq.subvec(3,5).zeros();

		double zeta = 1.2; //0.7 for slightly underdamped. Set 1 for critically damped
		C_extra = zeta * ( 2.0 * arma::sqrt(M_d * K_eq) );

		v_ref = arma::inv(M_d / robot->cycle + arma::max(C_d, arma::diagmat(C_extra))) * (M_d * v_ref / robot->cycle + ati_forces_ +  F_v);
	
		// v_ref.subvec(3,5).fill(0.0); //disable compliance in rotation
		// saturateVelocity(index);

		U_v += arma::diagmat(-F_v) * v_ref * robot->cycle; 

		//reset Uv just to monitor what is going on
		if (pConstraints->getPerformanceIndex() > 1.2 * w_thT)
			U_v.zeros();

		qdot_ref = Jinv * v_ref; //differential inverse kinematics 
		// qdot_ref.fill(0.0); //set zero if no task motion is commanded

		//nullsapce strategy (A needs to be calculated wrt the joint values)
		// qdot_ref += ( arma::eye<arma::mat>(7,7) - J.t() * Jinv.t() ) * ( nullspace_gain * A - nullspace_damping * qdot );

		//simple nullspace strategy. just rotate a joint
		// arma::vec qdot_null; qdot_null.zeros(7); qdot_null(4) = -0.02;
		// qdot_ref += ( arma::eye<arma::mat>(7,7) - J.t() * Jinv.t() ) * ( nullspace_gain * qdot_null /*- nullspace_damping * qdot*/ );

		q_ref += qdot_ref * robot->cycle;
	}	
}

void PerfConstraintsLWR::command()
{
	if (use_impedance)
		robot->setJointTorque(u);
	else
		robot->setJointPosition(q_ref);
}

/**
 * @brief printForLogging, prints logging variables
 */
void PerfConstraintsLWR::print_to_monitor()
{
	if (fmod(time, .15) < 0.001) { //plot every once in a while

		cout << "T:" << std::fixed << std::setprecision(1) << time; // << " [" << std::fixed << std::setprecision(1) <<robot->cycle*1000. << "ms]";
		cout << " w:" << std::setprecision(2) << std::setw(4) << pConstraints->getPerformanceIndex();
		
		cout << " F_h:[";
		for (int i=0; i<3; i++)
			cout << std::setw(5) << std::fixed << std::setprecision(1) << ati_forces_(i) << " ";
		cout << "] ";

		cout << " F_v:[";
		for (int i=0; i<3; i++)
			cout << std::setw(5) << std::fixed << std::setprecision(1) << F_v(i) << " ";
		cout << "] ";

		cout << " C_extra:[";
		for (int i=0; i<6; i++)
			cout << std::setw(5) << std::fixed << std::setprecision(1) << C_extra(i) << " ";
		cout << "] ";

		// cout << " U_v: " << std::setprecision(3) << arma::sum(U_v.subvec(0,2)) << " [";
		// for (int i=0; i<3; i++)
		// 	cout << std::setw(5) << std::fixed << std::setprecision(3) << U_v(i) << " ";
		// cout << "] ";
		

		cout << endl; //empty line
	}
}

void PerfConstraintsLWR::write_to_file(){
	if (write2file) {// && fmod(time, 0.01) < 0.001) { //enters when a 'filename' param is given in config.yml AND every few loops (smaller filesize)
		outStream << std::fixed << std::setprecision(6) << time << ","; //column 1

		for (int i=0; i<3; i++)
			outStream << std::fixed << std::setprecision(6) << p(i)<< ","; //2:4

		for (int i=0; i<7; i++)
			outStream << std::fixed << std::setprecision(6) << q.at(i) << ","; //5:11 joint values

		for (int i=0; i<6; i++)
			outStream << std::fixed << std::setprecision(6) << ati_forces_.at(i) << ","; //12:17 ATI forces wrt. the base frame

		for (int i=0; i<6; i++)
			outStream << std::fixed << std::setprecision(6) << v_ref.at(i) << ","; //18:23 reference velocity

		outStream << std::fixed << std::setprecision(6) << pConstraints->getPerformanceIndex() << ","; //24 w

		for (int i=0; i<6; i++)
			outStream << std::fixed << std::setprecision(6) << A.at(i) << ","; //25:30 gradient 

		for (int i=0; i<6; i++)
			outStream << std::fixed << std::setprecision(6) << F_v.at(i) << ","; //31:36 constraint forces 

		for (int i=0; i<6; i++)
			outStream << std::fixed << std::setprecision(6) << U_v.at(i) << ","; //37:42 energy 

		outStream << std::fixed << std::setprecision(6) << pConstraints->getPerformanceIndex(1) << ","; //24 w

		outStream << endl; //newline
	}
}

void PerfConstraintsLWR::init() {
	//move to initial pose
	arma::vec qT;
	// qT << 0.0 << 0.4014 << 0.0 << -1.3963 << 0.0 << 1.0835 << 0.0;
	// qT << 0.0 << 0.9599 << 0.0 << -1.85 << 0.1 << -1.815 << 0.0; 
	qT << -M_PI/2.0 << -0.4014 << 0.0 << 1.3963 << 0.1 << -1.34835 << 0.0;
	robot->setMode(arl::robot::Mode::POSITION_CONTROL); //position mode
	cout << "Moving to start configuration..." << endl;
	robot->setJointTrajectory(qT, 6.0);

	//Generate trajectory to follow
    pose = robot->getTaskPose().matrix().toArma().submat(0,0,2,3);
	p = pose.col(3);
	p_ref = p; //mirror current position
	pose_ref = pose;

	Qd = arl::math::rotToQuat(pose.submat(0, 0, 2, 2)); //[keep Q desired on the initial one]
	Quat_ref = Qd; //enable this in the case of pure orientation control
	Qprev = Qd;

	//Save starting configuration
    q_ref = robot->getJointPosition().toArma();
}

bool PerfConstraintsLWR::run()
{
	// std::thread waitInput(&PerfConstraintsLWR::keyboardControl, this);

	init();

	setImpedanceParams();

	if (use_impedance) { //change mode only if we want to send torques

		//Switch to impedance control
		robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
	}
	std::cout << "Starting controller!\n";
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

