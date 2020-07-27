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
	PerfConstraintsLWR(std::shared_ptr<arl::robot::Robot> robot, std::shared_ptr<arl::robot::Sensor> ft_sensor);
	void readParameters();
	void setImpedanceParams();

	void measure();
	void update();
	void command();

	void print_to_monitor();
	void write_to_file();

	// void keyboardControl();
	void init();
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
	arma::vec qdot, u, u_x, u_n, p, p_ref, p_init, p_ref_raw, v_ref, Q, Qprev, xdot, xdotRaw, Qd, e_p,  e_o,  pdot, quatDiff, forces, forces_filtered;
	arma::vec q, q_ref, pose_vec, Quat_ref, omega_ref, qdot_ref, F_v;
	arma::mat pose, pose_ref, J, Jinv, JinvW, M, Minv, K_d, C_d, M_d, R;
	arma::vec K_eq; 
	
	double w_thT, w_crT, w_thR, w_crR, lambda;

	std::string filename, inputfile;
	std::ofstream outStream;
	bool write2file, use_impedance;
	std::shared_ptr<PC> pConstraints;

	double stiff_transl, stiff_rot, damp_transl, damp_rot, inertia_transl, inertia_rot; //impedance/admittance free-space
	double nullspace_gain, nullspace_damping;
	arma::vec A;
	
	std::shared_ptr<arl::robot::Sensor> ft_sensor_;
	arma::mat::fixed<6,6> task_orientation_transform;
	arma::vec::fixed<6> ati_forces_, ati_forces_tool;
	
};

