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
	, msg_force_data_( 6, 0 )
	, msg_force_data_compensated_( 6, 0 )
	, msg_wrench_()
	, msg_pose_data_(6, 0)
	, msg_pose_()
	, msg_sensor_compensation_params_(6,0)
	, msg_stiff_force_z_( 0, 0 )
	, msg_stiff_pose_z_()
	, sensor_compensation_(false)
	, sensor_compensation_type_(0)
	, sensor_compensation_sample_size_(0)
	, reduce_zero_noice_(false)
	, reduce_zero_noice_cutoff_(0.0)
	, stiffness_calculation_(0)
	, stiffness_(0)
	{
		// Add ports. (addEventPort() for a port that wakes up the activity)
		addPort("in_force_data", in_force_data_).doc( "The force data from the force sensor");
		addPort("out_force_data", out_force_data_).doc( "The force data after processing");
		addPort("out_wrench", out_wrench_).doc( "The force data in wrench format");

		addPort("in_pose_data", in_pose_data_).doc( "The pose data");
		addPort("out_pose_data", out_pose_data_).doc( "The pose data after processing");
		addPort("out_pose", out_pose_).doc( "The pose data in twist format");

		addProperty("sensor_compensation",sensor_compensation_).doc("Flag to control sensor compensation");
		addProperty("sensor_compensation_type", sensor_compensation_type_).doc("Type of sensor compensation");
		addProperty("sensor_compensation_sample_size", sensor_compensation_sample_size_).doc("Amount of samples for auto sensor compensation");
		addProperty("sensor_compensation_params_calculated", sensor_compensation_params_calculated_).doc("Flag to show if the compensation parameters are already calculated");
		addProperty("reduce_zero_noice", reduce_zero_noice_).doc("Flag to enable the reduction of zero noice");
		addProperty("reduce_zero_noice_cutoff", reduce_zero_noice_cutoff_).doc("Reduce zero noice cutoff");
		addProperty("stiffness_calculation", stiffness_calculation_).doc("Flag to regulate stiffness calculation");
		addProperty("stiffness", stiffness_).doc("Stiffness parameter");

		std::vector<std::vector<double> > stiffness_data;


		// Show messages to the output ports to guarantee real-timeness.
		out_force_data_.setDataSample( msg_force_data_);
		out_pose_data_.setDataSample( msg_pose_data_);
		out_wrench_.setDataSample( msg_wrench_);
		out_pose_.setDataSample( msg_pose_);
	}

	/**
	* This function is for the configuration code.
	* Return false to abort configuration.
	*/
	bool GielComponent::configureHook()
	{
		//return this->setPeriod(0.001); // set to 1000Hz execution mode.
		//return true;
	}

	/**
	* This function is for the application's start up code.
	* Return false to abort start up.
	*/
	bool GielComponent::startHook( )
	{

		// Check validity of (all) Ports:
		if ( !in_force_data_.connected() ) {
			// No connection was made, can't do my job !
			return false;
		}

		// Reset input ports.
		in_force_data_.clear();
		in_pose_data_.clear();

		// Complete startHook()
		return true;
	}

	/*
	* Update hook of the OROCOS component.
	*/
	void GielComponent::updateHook( )
	{
		// Read the force data
		in_force_data_.read( msg_force_data_ );
		in_pose_data_.read( msg_pose_data_);

		// Calculate sensor compensation Parameters
		if (sensor_compensation_params_calculated_ == 1) {
			msg_sensor_compensation_params_ = sensorCompensationParams(msg_force_data_, sensor_compensation_type_, sensor_compensation_sample_size_);
			sensor_compensation_params_calculated_ = 2;
		}

		// Apply Sensor compensation
		if (sensor_compensation_) {
			msg_force_data_compensated_ = sensorCompensation(msg_force_data_, msg_sensor_compensation_params_);
			//if (reduce_zero_noice_) {
				//msg_force_data_compensated_ = reduceZeroNoice(msg_force_data_compensated_, reduce_zero_noice_cutoff_);
			//}
		}
		else {
			msg_force_data_compensated_ = msg_force_data_;
			//cout << "No sensor compensation";
		}

		// Reduce Zero Noice
		if (reduce_zero_noice_) {
			msg_force_data_compensated_ = reduceZeroNoice(msg_force_data_compensated_, reduce_zero_noice_cutoff_);
		}

		msg_stiff_pose_z_.push_back(msg_pose_data_[0]);

		// Calculate stiffness
		if (stiffness_calculation_ == 1) {
			msg_stiff_force_z_.push_back(msg_force_data_compensated_[2]);
			msg_stiff_pose_z_.push_back(msg_pose_data_[2]);
		}
		else if (stiffness_calculation_ == 2) {
			stiffness_ = calc_stiffness(msg_stiff_force_z_, msg_stiff_pose_z_);
		}


		// Create the wrench
		msg_wrench_.force.x = msg_force_data_compensated_[0];
		msg_wrench_.force.y = msg_force_data_compensated_[1];
		msg_wrench_.force.z = msg_force_data_compensated_[2];
		msg_wrench_.torque.x = msg_force_data_compensated_[3];
		msg_wrench_.torque.y = msg_force_data_compensated_[4];
		msg_wrench_.torque.z = msg_force_data_compensated_[5];

		// Create the twist
		msg_pose_.linear.x = msg_pose_data_[0];
		msg_pose_.linear.y = msg_pose_data_[1];
		msg_pose_.linear.z = msg_pose_data_[2];
		msg_pose_.angular.x = msg_pose_data_[3];
		msg_pose_.angular.y = msg_pose_data_[4];
		msg_pose_.angular.z = msg_pose_data_[5];


		// Write force data to output-port
		out_force_data_.write(msg_force_data_compensated_);
		out_wrench_.write(msg_wrench_);
		out_pose_data_.write(msg_pose_data_);
		out_pose_.write(msg_pose_);

	}

	/*
	* Stop hook of the OROCOS component.
	*/
	void GielComponent::stopHook( )
	{

	}

	/*
	* Cleanup hook of the OROCOS component.
	*/
	void GielComponent::cleanupHook( )
	{
		//
	}

} // End of namespace.

ORO_CREATE_COMPONENT( gcomp::GielComponent )
