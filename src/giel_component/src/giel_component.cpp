#include "giel_component.hpp"
#include <rtt/Component.hpp>
#include <unistd.h>

#include <iomanip>
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <vector>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

using namespace RTT;
using namespace Eigen;


namespace gcomp {

	/**
	* Constructor of the OROCOS component.
	*/
	GielComponent::GielComponent( std::string const& _name ) : TaskContext( _name, PreOperational )
	, msg_force_data_(6,0)
	, msg_pose_data_(6,0)
	, msg_pose_pre_(6,0)
	, msg_wrench_()
	, msg_pose_()
	, msg_sensor_compensation_params_(6,0)
	, msg_stiff_force_z_()
	, msg_stiff_pose_z_()
	, pr_sensor_compensation_(false)
	, pr_sensor_compensation_sample_size_(0)
	, pr_sensor_compensation_flag_(0)
	, pr_stiffness_calculation_flag_(0)
	, pr_calculated_stiffness_(0.0)
	, pr_stiffness_force_start_(0.0)
    , pr_stiffness_force_stop_(0.0)
	, pr_sensor_tool_transx_(0)
	, pr_sensor_tool_transy_(0)
	, pr_sensor_tool_transz_(0)
	, pr_sensor_tool_rotx_(0)
	, pr_sensor_tool_roty_(0)
	, pr_sensor_tool_rotz_(0)
	, pr_sensor_tool_transformation_(6,0)
	, msg_transition_flag_(0)
	, pr_transition_flag_(0)
	, pr_scanning_flag_(false)
	{
		// Force and Pose ports.
		addPort("in_force_data", in_force_data_).doc( "The force data from the force sensor");
		addPort("out_force_data", out_force_data_).doc( "The force data after processing");
		addPort("out_wrench", out_wrench_).doc( "The force data in wrench format");
		addPort("out_stiffness_wrench", out_stiffness_wrench_).doc( "The force data in wrench format during the stiffness calculation state");
		addPort("out_scanning_wrench", out_scanning_wrench_).doc( "The force data in wrench format during the scanning state");

		addPort("in_pose_data", in_pose_data_).doc( "The pose data");
		addPort("out_pose_data", out_pose_data_).doc( "The pose data after processing");
		addPort("out_pose", out_pose_).doc( "The pose data in twist format");
		addPort("out_stiffness_pose", out_stiffness_pose_).doc( "The pose data in twist format during the stiffness calculation state");
		addPort("out_scanning_pose_", out_scanning_pose_).doc( "The force data in wrench format during the scanning state");

		// State transition port and flag
		addPort("out_transition_flag", out_transition_flag_).doc( "Port to publish the flag to show state transitions");
		addProperty("pr_transition_flag", pr_transition_flag_).doc( "Flag to show state transitions");
		addProperty("pr_scanning_flag", pr_scanning_flag_).doc( "Flag the system is scanning");

		// Properties for the transformation from sensor-frame to tool-frame
		addProperty("pr_sensor_tool_transx", pr_sensor_tool_transx_).doc("Delta x between sensor-frame and tool-frame");
		addProperty("pr_sensor_tool_transy", pr_sensor_tool_transy_).doc("Delta y between sensor-frame and tool-frame");
		addProperty("pr_sensor_tool_transz", pr_sensor_tool_transz_).doc("Delta z between sensor-frame and tool-frame");
		addProperty("pr_sensor_tool_rotx", pr_sensor_tool_rotx_).doc("Tranformation angle arround the x-axis of the sensor frame to the tool-frame");
		addProperty("pr_sensor_tool_roty", pr_sensor_tool_roty_).doc("Tranformation angle arround the y-axis of the sensor frame to the tool-frame");
		addProperty("pr_sensor_tool_rotz", pr_sensor_tool_rotz_).doc("Tranformation angle arround the z-axis of the sensor frame to the tool-frame");
		addProperty("pr_sensor_tool_transformation", pr_sensor_tool_transformation_).doc("Array containing all framo transformation data");

		// Properties for sensor compensation
		addProperty("pr_sensor_compensation", pr_sensor_compensation_).doc("Bool to activate sensor compensation");
		addProperty("pr_sensor_compensation_sample_size", pr_sensor_compensation_sample_size_).doc("Amount of samples for auto sensor compensation");
		addProperty("pr_sensor_compensation_flag", pr_sensor_compensation_flag_).doc("Flag to show if the compensation parameters are already calculated");

		// Properties for stiffness calculation
		addProperty("pr_stiffness_calculation_flag", pr_stiffness_calculation_flag_).doc("Flag to regulate stiffness calculation");
		addProperty("pr_calculated_stiffness", pr_calculated_stiffness_).doc("Stiffness parameter");
		addProperty("pr_stiffness_force_start", pr_stiffness_force_start_).doc("Force treshold to start recording for stiffness calculation ");
		addProperty("pr_stiffness_force_stop", pr_stiffness_force_stop_).doc("Force treshold to stop recording for stiffness calculation ");

		// Show messages to the output ports to guarantee real-timeness.
		out_force_data_.setDataSample(msg_force_data_);
		out_wrench_.setDataSample(msg_wrench_);
		out_stiffness_wrench_.setDataSample(msg_wrench_);
		out_scanning_wrench_.setDataSample(msg_wrench_);

		out_pose_data_.setDataSample( msg_pose_data_);
		out_pose_.setDataSample(msg_pose_);
		out_stiffness_pose_.setDataSample(msg_pose_);
		out_scanning_pose_.setDataSample(msg_pose_);

		out_transition_flag_.setDataSample(msg_transition_flag_);
	}

	/*************************************************************************
	* CONFIGURE HOOK
	* This function is for the configuration code.
	* Return false to abort configuration.
	*************************************************************************/
	bool GielComponent::configureHook()
	{
		//return this->setPeriod(0.002); // set to 500Hz execution mode.
		//return true;
	}

	/*************************************************************************
	 * START HOOK
	 * This function is for the application's start up code.
	 * Return false to abort start up.
	*************************************************************************/
	bool GielComponent::startHook( )
	{
		// Check validity of the force input
		if ( !in_force_data_.connected() ) {
			return false;
		}
		// Check validity of the pose input
		if ( !in_pose_data_.connected() ) {
			return false;
		}

		// Reset neccesary properties
		pr_sensor_compensation_ = false;
		pr_sensor_compensation_flag_ = 0;
		pr_stiffness_calculation_flag_ = 0;
		pr_transition_flag_ = 0;
		pr_scanning_flag_ = false;

		// Set neccesary Application_properties
		pr_sensor_tool_transformation_[0] = pr_sensor_tool_transx_;
		pr_sensor_tool_transformation_[1] = pr_sensor_tool_transy_;
		pr_sensor_tool_transformation_[2] = pr_sensor_tool_transz_;
		pr_sensor_tool_transformation_[3] = pr_sensor_tool_rotx_;
		pr_sensor_tool_transformation_[4] = pr_sensor_tool_roty_;
		pr_sensor_tool_transformation_[5] = pr_sensor_tool_rotz_;

		// Reset input ports
		in_force_data_.clear();
		in_pose_data_.clear();

		// Complete startHook()
		return true;
	}

	/*************************************************************************
	 * UPDATE HOOK
	 * Update hook of the OROCOS component.
	*************************************************************************/
	void GielComponent::updateHook( )
	{
		// GET INPUT DATA
		in_force_data_.read( msg_force_data_ );
		in_pose_data_.read( msg_pose_data_);

		// MAKE POSE DATA CONTINUOUS
		// If the pose data suddenly is zero, assume pose data is not received and the robot stays in the same pose.
		if (detect_zero_pose_data(msg_pose_data_)){
			msg_pose_data_ = msg_pose_pre_;
		}
		else{
			msg_pose_pre_ = msg_pose_data_;
		}

		// SENSOR FRAME TO TOOL FRAME TRANSFORMATION
		msg_force_data_ = sensor_to_tool_frame(msg_force_data_, pr_sensor_tool_transformation_);

		// SENSOR COMPENSATION: Calculate sensor compensation Parameters
		if (pr_sensor_compensation_flag_ == 1){
			msg_sensor_compensation_params_ = sensorCompensationParams(msg_force_data_, pr_sensor_compensation_sample_size_);
			pr_sensor_compensation_flag_ = 2;
		}

		// SENSOR COMPENSATION: Apply Sensor compensation
		if (pr_sensor_compensation_){
			msg_force_data_ = sensorCompensation(msg_force_data_, msg_sensor_compensation_params_);
		}

		// CREATE WRENCH AND POSE (TWIST)
		// Create the wrench
		msg_wrench_.force.x = msg_force_data_[0];
		msg_wrench_.force.y = msg_force_data_[1];
		msg_wrench_.force.z = msg_force_data_[2];
		msg_wrench_.torque.x = msg_force_data_[3];

		msg_wrench_.torque.y = msg_force_data_[4];
		msg_wrench_.torque.z = msg_force_data_[5];
		// Create the twist
		msg_pose_.linear.x = msg_pose_data_[0];
		msg_pose_.linear.y = msg_pose_data_[1];
		msg_pose_.linear.z = msg_pose_data_[2];
		msg_pose_.angular.x = msg_pose_data_[3];
		msg_pose_.angular.y = msg_pose_data_[4];
		msg_pose_.angular.z = msg_pose_data_[5];

		// STIFFNESS CALCULATION
		// Stiffness_calculation_ = 0 --> No stiffness calculation done.
		// Stiffness_calculation_ = 1 --> Stiffness calculation is being excecuted
		// Stiffness_calculation_ = 2 --> Stiffness calculation is completed
		if (pr_stiffness_calculation_flag_ == 1) {
			if (msg_force_data_[2] <= pr_stiffness_force_start_ && msg_force_data_[2] >= pr_stiffness_force_stop_) {
				out_stiffness_wrench_.write(msg_wrench_);
				out_stiffness_pose_.write(msg_pose_);
				msg_stiff_force_z_.push_back(msg_force_data_[2]);
				msg_stiff_pose_z_.push_back(msg_pose_data_[2]);
			}
		}
		else if(pr_stiffness_calculation_flag_ == 2){
			pr_calculated_stiffness_= calc_stiffness(msg_stiff_force_z_, msg_stiff_pose_z_);
			if(pr_calculated_stiffness_ != pr_calculated_stiffness_){
				pr_calculated_stiffness_ = 0;
			}
		}

		// GATHER SCANNING DATA
		if (pr_scanning_flag_ == true){
			out_scanning_wrench_.write(msg_wrench_);
			out_scanning_pose_.write(msg_pose_);
		}

		// WRITE OUTPUT
		msg_transition_flag_ = pr_transition_flag_;
		out_transition_flag_.write(msg_transition_flag_);

		out_force_data_.write(msg_force_data_);
		out_wrench_.write(msg_wrench_);
		out_pose_data_.write(msg_pose_data_);
		out_pose_.write(msg_pose_);
	}

	/*************************************************************************
	 * STOP HOOK
	 * Stophook of the OROCOS component.
	*************************************************************************/
	void GielComponent::stopHook( )
	{
		//
	}

	/*************************************************************************
	 * CLEANUP HOOK
	 * Cleanup hook of the OROCOS component.
	*************************************************************************/
	void GielComponent::cleanupHook( )
	{
		//
	}

} // End of namespace.

ORO_CREATE_COMPONENT( gcomp::GielComponent )
