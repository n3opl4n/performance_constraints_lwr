/**
 * Copyright (C) 2020 AUTH-ARL
 */

#include <lwr_robot/robot.h>
#include <autharl_core/robot/ros_model.h>
#include <autharl_core/viz/ros_state_publisher.h>
#include <controller.h>
#include <autharl_ati_sensor/autharl_ati_sensor.h>
#include <ros/ros.h>
#include <memory>
#include <iostream>


int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "deformation_node");
  
  auto model = std::make_shared<arl::robot::ROSModel>("/robot_description");

   // Create a robot sensor, use can use a real robot also
  auto sensor = std::make_shared<ati::sensor::Sensor>("ATI Sensor", "lwr_arm_7_link");

   // Create generic robot
  std::shared_ptr<arl::robot::Robot> robot;
  
  // Initialize generic robot with the kuka-lwr model
  robot.reset(new arl::lwr::Robot(model, "Kuka Robot"));
  ROS_INFO_STREAM("Robot created successfully.");

  auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot);

  // std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);
  
  // // Create controller instance for the kuka-lwr robot
  std::unique_ptr<PerfConstraintsLWR> controller(new PerfConstraintsLWR(robot, sensor));
  
  // auto rviz = std::make_shared<arl::viz::RosStatePublisher>(robot);
  // std::thread rviz_thread(&arl::viz::RosStatePublisher::run, rviz);
  
  controller->run();
  
  ros::spin();
  // rviz_thread.join();
  return 0;
}