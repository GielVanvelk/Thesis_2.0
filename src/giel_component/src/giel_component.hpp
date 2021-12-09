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
		RTT::OutputPort < std::vector < double > >   	out_force_data_;
		RTT::OutputPort < std::vector < double > >   	out_pose_data_;

		RTT::OutputPort < geometry_msgs::Wrench  >   	out_wrench_;
		RTT::OutputPort < geometry_msgs::Twist   >   	out_pose_;

		RTT::OutputPort < geometry_msgs::Wrench  >   	out_stiffness_wrench_;
		RTT::OutputPort < geometry_msgs::Twist   >   	out_stiffness_pose_;

		RTT::OutputPort < geometry_msgs::Wrench  >   	out_scanning_wrench_;
		RTT::OutputPort < geometry_msgs::Twist   >   	out_scanning_pose_;

		RTT::OutputPort < int >   						out_transition_flag_;

	private:

		// MESSAGES: FORCE
		std::vector < double >       					msg_force_data_;
		std::vector < double >       					msg_stiff_force_z_;
		geometry_msgs::Wrench                           msg_wrench_;

		// MESSAGES: POSE
		std::vector < double >                          msg_pose_data_;
		std::vector < double >                          msg_pose_pre_;
		std::vector < double >       					msg_stiff_pose_z_;
		geometry_msgs::Twist                            msg_pose_;

		// MESSAGES: OTHER
		std::vector < double >       					msg_sensor_compensation_params_;
		int    											msg_transition_flag_;

		// PROPERTIES: SENSOR COMPENSATION
		bool  											pr_sensor_compensation_;
		int	 											pr_sensor_compensation_sample_size_;
	    int 											pr_sensor_compensation_flag_; // 0:Not Calculated, 1:Start Calculation, 2:Calculated

		// PROPERTIES: STIFFNESS CALCULATION
		int												pr_stiffness_calculation_flag_; // 0:Not Calculated, 1:Save data, 2:Calculate Stiffness
		double										    pr_calculated_stiffness_;
		double										    pr_stiffness_force_start_;
		double										    pr_stiffness_force_stop_;

		// PROPERTIES: TRANSFORMATION SENSOR- TO TOOL-FRAME
		double      									pr_sensor_tool_transx_;
		double      									pr_sensor_tool_transy_;
		double      									pr_sensor_tool_transz_;
		double      									pr_sensor_tool_rotx_;
		double      									pr_sensor_tool_roty_;
		double      									pr_sensor_tool_rotz_;
		std::vector < double >       					pr_sensor_tool_transformation_;

		// PROPERTIES: STATE TRANSITION FLAG
		int 					  						pr_transition_flag_;
		bool					  						pr_scanning_flag_;
	};

	bool detect_zero_pose_data(std::vector < double > pose_data);
	std::vector < double > sensor_to_tool_frame(std::vector < double > force_data_sensor_frame, std::vector < double > transformation_params);

	std::vector < double > sensorCompensationParams(std::vector < double > force_data, int sample_size);
	std::vector < double > autoSensorCompensationParams(std::vector < double > force_data, int sample_size);
	std::vector < double > sensorCompensation(std::vector < double > force_data, std::vector < double > compensation_params);
	std::vector < double > reduceZeroNoice(std::vector < double > data, double cutoff);
	double average_vector(std::vector < double > v);

	double calc_stiffness ( std::vector < double > force, std::vector < double > pose );
	double sensor_filtering(std::vector < double > data, int size );
}
#endif
