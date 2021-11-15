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

		std::vector < double >       					points;
		std::vector < double >       					x;
		std::vector < double >       					y;
		std::vector < double >       					z;


	protected:

		// INPUT-Ports
		RTT::InputPort < std::vector < double > >       in_force_data_;
		RTT::InputPort < std::vector < double > >       in_pose_data_;

		// OUTPUT-Ports
		RTT::OutputPort < geometry_msgs::Wrench  >   	out_wrench_;
		RTT::OutputPort < geometry_msgs::Twist   >   	out_pose_;

		RTT::OutputPort < geometry_msgs::Wrench  >   	out_stiffness_wrench_;
		RTT::OutputPort < geometry_msgs::Twist   >   	out_stiffness_pose_;

		RTT::OutputPort < std::vector < double > >   	out_force_data_;
		RTT::OutputPort < std::vector < double > >   	out_pose_data_;

		RTT::OutputPort < int >   						out_state_transition_flag_;

	private:
		// Messages
		std::vector < double >       					msg_force_data_;
		geometry_msgs::Wrench                           msg_wrench_;

		std::vector < double >                          msg_pose_data_;
		std::vector < double >                          msg_pose_pre_;
		geometry_msgs::Twist                            msg_pose_;

		std::vector < double >       					msg_sensor_compensation_params_;

		std::vector < double >       					msg_stiff_pose_z_;
		std::vector < double >       					msg_stiff_force_z_;

		std::vector<double> 							forceZ_data_compensated_;

		int    											msg_state_transition_flag_;



		// Properties for sensor compensation
		bool  											sensor_compensation_;
        int 											sensor_compensation_type_;
		int	 											sensor_compensation_sample_size_;
	    int 											sensor_compensation_params_calculated_; // 0:Not Calculated, 1:Start Calculation, 2:Calculated
		bool											reduce_zero_noice_;
		double 											reduce_zero_noice_cutoff_;

		// Properties for FIFO Buffer
		bool 											use_fifo_buffer_;
		int 											fifo_buffer_size_;

		// Properties for stiffness calculation
		int												stiffness_calculation_;
		double										    stiffness_;
		double										    stiffness_calc_force_start_;
		double										    stiffness_calc_force_stop_;

		// Properties for the transformation from sensor-frame to tool-frame
		double      									sensor_tool_transx_;
		double      									sensor_tool_transy_;
		double      									sensor_tool_transz_;
		double      									sensor_tool_rotx_;
		double      									sensor_tool_roty_;
		double      									sensor_tool_rotz_;
		std::vector < double >       					sensor_tool_transformation_;

		int 					  						pr_state_transition_flag_;
	};

	// Function for compensating the force data (gravity)
	std::vector < double > sensor_to_tool_frame(std::vector < double > force_data_sensor_frame, std::vector < double > transformation_params);
	bool detect_zero_pose_data(std::vector < double > pose_data);
	std::vector < double > sensorCompensation(std::vector < double > force_data, std::vector < double > compensation_params);
	std::vector < double > sensorCompensationParams(std::vector < double > force_data, int comp_type ,int sample_size);
	std::vector < double > autoSensorCompensationParams(std::vector < double > force_data, int sample_size);
	std::vector < double > reduceZeroNoice(std::vector < double > data, double cutoff);
	double average_vector(std::vector < double > v);
	double calc_stiffness ( std::vector < double > force, std::vector < double > pose );
	double sensor_filtering(std::vector < double > data, int size);
	//void safe_to_bag();


} // End of namespace usconnector.


#endif
