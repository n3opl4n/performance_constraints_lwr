/**
 * Copyright (C) 2020 AUTH-ARL
 */

#include <autharl_core/eigen_plugins.h>
#include <lwr_robot/robot.h>
#include <autharl_core/robot/controller.h>
#include <autharl_core/math.h>
#include <ros/ros.h>
#include <memory>
#include <vector>
#include <fstream>
#include <thread>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <performance_constraints/performanceConstraints.h>

class PerfConstraintsLWR : public arl::robot::Controller
{
public:
	PerfConstraintsLWR(std::shared_ptr<arl::robot::Robot> robot);
	void readParameters();
	void setImpedanceParams();

	void measure();
	void update();
	void command();

	void print_to_monitor();
	void write_to_file();

	/**
	 * Progressive automation starts from here. The robot moves to an initial configuration with position control and then switches to Torque control
	 * @return true when terminates successfully
	 */
	bool run();

	/**
	 * Check if a stop of the program has been requested
	 * @return true if the program is requested to terminate (e.g. when Enter is pressed)
	 */
	// bool stop();


	
private:
	double time;
	ros::NodeHandle nh_;
	arma::vec qdot, u, u_x, u_n, p, p_ref, p_init, p_ref_raw, Q, Qprev, xdot, xdotRaw, Qd, e_p,  e_o,  pdot, quatDiff, forces, forces_filtered;
	arma::vec q, q_ref, pose_vec, Quat_ref, omega_ref;
	arma::mat pose, pose_ref, J, Jsym, K_imp, D_imp, R, dmp_ref, Pose_history;
	arma::vec Pose_var, ww, forces_ref; 
	
	std::string filename, inputfile;
	std::ofstream outStream;
	bool write2file;
	std::shared_ptr<PC> pConstraints;

	double stiff_transl, stiff_rot, damp_transl, damp_rot; //impedance free-space
	
};

