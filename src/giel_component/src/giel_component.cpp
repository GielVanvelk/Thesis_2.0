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
	, forceZ_data_compensated_()

	, sensor_compensation_(false)
	, sensor_compensation_type_(0)
	, sensor_compensation_sample_size_(0)
	, sensor_compensation_params_calculated_(0)
	, reduce_zero_noice_(false)
	, reduce_zero_noice_cutoff_(0.0)

	, use_fifo_buffer_(false)
	, fifo_buffer_size_(0)

	, stiffness_calculation_(0)
	, stiffness_(0.0)
	, stiffness_calc_force_start_(0.0)
    , stiffness_calc_force_stop_(0.0)

	, sensor_tool_transx_(0)
	, sensor_tool_transy_(0)
	, sensor_tool_transz_(0)
	, sensor_tool_rotx_(0)
	, sensor_tool_roty_(0)
	, sensor_tool_rotz_(0)
	, sensor_tool_transformation_(6,0)

	, msg_state_transition_flag_(0)
	, pr_state_transition_flag_(0)
	{
		// Add ports. (addEventPort() for a port that wakes up the activity)
		addPort("in_force_data", in_force_data_).doc( "The force data from the force sensor");
		addPort("out_force_data", out_force_data_).doc( "The force data after processing");
		addPort("out_wrench", out_wrench_).doc( "The force data in wrench format");
		addPort("out_stiffness_wrench", out_stiffness_wrench_).doc( "The force data in wrench format during the stiffness state");

		addPort("in_pose_data", in_pose_data_).doc( "The pose data");
		addPort("out_pose_data", out_pose_data_).doc( "The pose data after processing");
		addPort("out_pose", out_pose_).doc( "The pose data in twist format");
		addPort("out_stiffness_pose", out_stiffness_pose_).doc( "The pose data in twist format during the stiffness state");

		addPort("out_state_transition_flag", out_state_transition_flag_).doc( "Flag to show state transitions");
		addProperty("pr_state_transition_flag", pr_state_transition_flag_).doc( "Flag to show state transitions");

		// Properties for the transformation from sensor-frame to tool-frame
		addProperty("sensor_tool_transx", sensor_tool_transx_).doc("Delta x between sensor-frame and tool-frame");
		addProperty("sensor_tool_transy", sensor_tool_transy_).doc("Delta y between sensor-frame and tool-frame");
		addProperty("sensor_tool_transz", sensor_tool_transz_).doc("Delta z between sensor-frame and tool-frame");
		addProperty("sensor_tool_rotx", sensor_tool_rotx_).doc("Tranformation angle arround the x-axis of the sensor frame to the tool-frame");
		addProperty("sensor_tool_roty", sensor_tool_roty_).doc("Tranformation angle arround the y-axis of the sensor frame to the tool-frame");
		addProperty("sensor_tool_rotz", sensor_tool_rotz_).doc("Tranformation angle arround the z-axis of the sensor frame to the tool-frame");
		addProperty("sensor_tool_transformation", sensor_tool_transformation_).doc("Tranformation parameters of the sensor frame to the tool-frame");

		// Properties for sensor compensation
		addProperty("sensor_compensation",sensor_compensation_).doc("Flag to control sensor compensation");
		addProperty("sensor_compensation_type", sensor_compensation_type_).doc("Type of sensor compensation");
		addProperty("sensor_compensation_sample_size", sensor_compensation_sample_size_).doc("Amount of samples for auto sensor compensation");
		addProperty("sensor_compensation_params_calculated", sensor_compensation_params_calculated_).doc("Flag to show if the compensation parameters are already calculated");
		addProperty("reduce_zero_noice", reduce_zero_noice_).doc("Flag to enable the reduction of zero noice");
		addProperty("reduce_zero_noice_cutoff", reduce_zero_noice_cutoff_).doc("Reduce zero noice cutoff");

		// Properties for FIFO Buffer
		addProperty("use_fifo_buffer",use_fifo_buffer_).doc("Flag to control the use of the fifo buffer");
		addProperty("fifo_buffer_size",fifo_buffer_size_).doc("Size of the fifo buffer");

		// Properties for stiffness calculation
		addProperty("stiffness_calculation", stiffness_calculation_).doc("Flag to regulate stiffness calculation");
		addProperty("stiffness", stiffness_).doc("Stiffness parameter");
		addProperty("stiffness_calc_force_start", stiffness_calc_force_start_).doc("Force treshold to start recording for stiffness calculation ");
		addProperty("stiffness_calc_force_stop", stiffness_calc_force_stop_).doc("Force treshold to stop recording for stiffness calculation ");
		//std::vector<std::vector<double> > stiffness_data;

		// Show messages to the output ports to guarantee real-timeness.
		out_force_data_.setDataSample( msg_force_data_);
		out_pose_data_.setDataSample( msg_pose_data_);
		out_wrench_.setDataSample( msg_wrench_);
		out_pose_.setDataSample( msg_pose_);
		out_stiffness_wrench_.setDataSample( msg_wrench_);
		out_stiffness_pose_.setDataSample( msg_pose_);
		out_state_transition_flag_.setDataSample( msg_state_transition_flag_);
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
		// Check validity of the input ports
		if ( !in_force_data_.connected() ) {
			// No connection was made, can't do my job !
			return false;
		}

		if ( !in_pose_data_.connected() ) {
			// No connection was made, can't do my job !
			return false;
		}

		// Reset neccesary properties
		sensor_compensation_ = 0;
		sensor_compensation_params_calculated_ = 0;
		stiffness_calculation_ = 0;
		use_fifo_buffer_ = false;
		pr_state_transition_flag_=0;

		// Set neccesary Application_properties
		sensor_tool_transformation_[0] = sensor_tool_transx_;
		sensor_tool_transformation_[1] = sensor_tool_transy_;
		sensor_tool_transformation_[2] = sensor_tool_transz_;
		sensor_tool_transformation_[3] = sensor_tool_rotx_;
		sensor_tool_transformation_[4] = sensor_tool_roty_;
		sensor_tool_transformation_[5] = sensor_tool_rotz_;

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
		// If the pose data suddenly is zero, assume pose data is not received
		// and the robot stays in the same pose.
		if (detect_zero_pose_data(msg_pose_data_)){
			msg_pose_data_ = msg_pose_pre_;
		}
		else{
			msg_pose_pre_ = msg_pose_data_;
		}

		// SENSOR FRAME TO TOOL FRAME TRANSFORMATION
		msg_force_data_ = sensor_to_tool_frame(msg_force_data_, sensor_tool_transformation_);


		// SENSOR COMPENSATION
		// Calculate sensor compensation Parameters
		if (sensor_compensation_params_calculated_ == 1){
			msg_sensor_compensation_params_ = sensorCompensationParams(msg_force_data_, sensor_compensation_type_, sensor_compensation_sample_size_);
			sensor_compensation_params_calculated_ = 2;
		}
		// Apply Sensor compensation
		if (sensor_compensation_){
			msg_force_data_ = sensorCompensation(msg_force_data_, msg_sensor_compensation_params_);
		}
		// Reduce Zero Noice
		if (reduce_zero_noice_){
			msg_force_data_ = reduceZeroNoice(msg_force_data_, reduce_zero_noice_cutoff_);
		}

		// SENSOR FILTERING
		if (use_fifo_buffer_== true){
			if (forceZ_data_compensated_.size() < fifo_buffer_size_ -1){
				forceZ_data_compensated_.push_back(msg_force_data_[2]);
			}
			else {
				for (int i = 0; i < fifo_buffer_size_ -1; i++){
					forceZ_data_compensated_[i] = forceZ_data_compensated_[i+1];
					forceZ_data_compensated_[fifo_buffer_size_-1] = msg_force_data_[2];
				}
			}
			msg_force_data_[2] = average_vector(forceZ_data_compensated_);
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


		/**
		// Create COSINUS
		for (int i = 0; i<= 100; i++){
			if(i ==0){
				points.push_back(0);
				x.push_back(-0.35);
				y.push_back(0);
				z.push_back(0.25);
			}
			else{
				points.push_back(1/i);
				x.push_back(-0.35);
				y.push_back(0);
				z.push_back(-1*cos(20/(i*2*3.1415)));
			}
		}
		// Write data to file
		std::ofstream myFile("cosinus.csv");
		myFile << "x";
		myFile.close();
		**/




		// STIFFNESS CALCULATION
		// Stiffness_calculation_ = 0 --> No stiffness calculation done.
		// Stiffness_calculation_ = 1 --> Stiffness calculation is being excecuted
		// Stiffness_calculation_ = 2 --> Stiffness calculation is completed
		if (stiffness_calculation_ == 1) {
			if (msg_force_data_[2] <= stiffness_calc_force_start_){
				out_stiffness_wrench_.write(msg_wrench_);
				out_stiffness_pose_.write(msg_pose_);
				msg_stiff_force_z_.push_back(msg_force_data_[2]);
				msg_stiff_pose_z_.push_back(msg_pose_data_[2]);
			}
		}
		else if(stiffness_calculation_ == 2){
			stiffness_= calc_stiffness( msg_stiff_force_z_, msg_stiff_pose_z_);
			if(stiffness_ != stiffness_){
				stiffness_ = 0;
			}
		}

		// WRITE OUTPUT
		out_force_data_.write(msg_force_data_);
		out_wrench_.write(msg_wrench_);
		out_pose_data_.write(msg_pose_data_);
		out_pose_.write(msg_pose_);

		msg_state_transition_flag_ = pr_state_transition_flag_;
		out_state_transition_flag_.write(msg_state_transition_flag_);
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
