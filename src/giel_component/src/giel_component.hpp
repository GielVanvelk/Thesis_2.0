#ifndef GIEL_COMPONENT_HPP
#define GIEL_COMPONENT_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <rtt/RTT.hpp>
#include <string>
#include <Eigen/Geometry>

#include <rtt/Component.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

namespace gcomp
{

	class GielComponent : public RTT::TaskContext
	{
	public:

		GielComponent( std::string const& _name );
		bool configureHook( );
		bool startHook( );
		void updateHook( );
		void stopHook( );
		void cleanupHook( );

	protected:

		// INPUT-Ports
		RTT::InputPort < std::vector < double > >       in_force_data_;
		RTT::InputPort < std::vector < double > >       in_pose_data_;

		// OUTPUT-Ports
		RTT::OutputPort < geometry_msgs::Wrench  >   	out_wrench_;
		RTT::OutputPort < geometry_msgs::Twist   >   	out_pose_;

		RTT::OutputPort < std::vector < double > >   	out_force_data_;
		RTT::OutputPort < std::vector < double > >   	out_pose_data_;

	private:
		// Messages
		std::vector < double >       					msg_force_data_;
		std::vector < double >       					msg_force_data_compensated_;
		geometry_msgs::Wrench                           msg_wrench_;

		std::vector < double >                          msg_pose_data_;
		geometry_msgs::Twist                            msg_pose_;

		std::vector < double >       					msg_sensor_compensation_params_;

		std::vector < double >       					msg_stiff_pose_z_;
		std::vector < double >       					msg_stiff_force_z_;

		// Property
		bool  											sensor_compensation_;
        int 											sensor_compensation_type_;
		int	 											sensor_compensation_sample_size_;
	    int 											sensor_compensation_params_calculated_; // 0:Not Calculated, 1:Start Calculation, 2:Calculated
		bool											reduce_zero_noice_;
		double 											reduce_zero_noice_cutoff_;
		int												stiffness_calculation_;
		int												stiffness_;
	};

	// Function for compensating the force data (gravity)
	std::vector < double > sensorCompensation(std::vector < double > force_data, std::vector < double > compensation_params);
	std::vector < double > sensorCompensationParams(std::vector < double > force_data, int comp_type ,int sample_size);
	std::vector < double > autoSensorCompensationParams(std::vector < double > force_data, int sample_size);
	std::vector < double > reduceZeroNoice(std::vector < double > data, double cutoff);
	double average_vector(std::vector < double > v);
	double calc_stiffness ( std::vector < double > force, std::vector < double > pose );
	//void safe_to_bag();

} // End of namespace usconnector.


#endif
