// Copyright  (C)  2013  Enea Scioni <enea dot scioni at unife dot it>,
// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 
// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>

// Author: Enea Scioni, Ruben Smits, Wilm Decre
// Maintainer: Enea Scioni, Ruben Smits, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef _FRI_COMPONENT_SIM_HPP_
#define _FRI_COMPONENT_SIM_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include  <lwr_fri_msgs/FriJointState.h>
#include  <lwr_fri_msgs/CartesianImpedance.h>
#include  <lwr_fri_msgs/FriJointImpedance.h>
#include  <lwr_fri_msgs/FriJointCommand.h>
#include  <lwr_fri_msgs/FriRobotData.h>
#include  <lwr_fri_msgs/FriRobotJntData.h>
#include <sensor_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <motion_control_msgs/typekit/Types.hpp>

#include <types/kuka_lwr_fri/friComm.h>

namespace lwr_fri {

using namespace RTT;

class FRIComponentSim: public RTT::TaskContext {
public:
	FRIComponentSim(const std::string& name);
	~FRIComponentSim();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

        //Extra methods/operation for emulation
        bool setFriState(const std::string& mode);
        bool setControlMode(const std::string& mode);
private:

	bool isPowerOn() { return m_msr_data.robot.power!=0; }

	tFriMsrData m_msr_data;
	tFriCmdData m_cmd_data;

	sensor_msgs::JointState m_joint_states;
	sensor_msgs::JointState my_joint_state;
	lwr_fri_msgs::FriJointState m_fri_joint_state;

	geometry_msgs::Pose m_cartPos;
	geometry_msgs::Twist m_cartTwist;
	geometry_msgs::Wrench m_cartWrench;

	lwr_fri_msgs::CartesianImpedance m_cartImp;

	OutputPort<tFriKrlData> port_from_krl;
	InputPort<tFriKrlData> port_to_krl;
	//Eigen::Matrix<double,7,7> m_massTmp; Not correct so useless

	/**
	 * events
	 */
	OutputPort<std::string> port_events;

	/**fri_create_socket
	 * statistics
	 */
	OutputPort<tFriRobotState> port_robot_state;
	OutputPort<tFriIntfState> port_fri_state;

	/**
	 * Current robot data
	 */
	OutputPort<sensor_msgs::JointState> port_joint_state;
	OutputPort<lwr_fri_msgs::FriJointState> port_fri_joint_state;

	OutputPort<geometry_msgs::Pose>  port_cart_pos_msr;
	OutputPort<KDL::Frame> port_cart_frame_msr;
	//OutputPort<geometry_msgs::Pose>  m_cmdCartPosPort;
	//OutputPort<geometry_msgs::Pose>  m_cmdCartPosFriOffsetPort;
	OutputPort<geometry_msgs::Wrench> port_cart_wrench_msr;
	RTT::OutputPort<KDL::Jacobian> jacobianPort;
	//RTT::OutputPort<Eigen::MatrixXd > massMatrixPort;

	lwr_fri_msgs::FriJointCommand m_fri_joint_command;
	motion_control_msgs::JointPositions m_joint_pos_command;
	motion_control_msgs::JointVelocities m_joint_vel_command;
	motion_control_msgs::JointEfforts m_joint_effort_command;
	lwr_fri_msgs::FriJointImpedance m_fri_joint_impedance;
	InputPort<lwr_fri_msgs::FriJointCommand> port_fri_joint_command;
	InputPort<motion_control_msgs::JointPositions> port_joint_pos_command;
	InputPort<motion_control_msgs::JointVelocities> port_joint_vel_command;
	InputPort<motion_control_msgs::JointEfforts> port_joint_effort_command;

	InputPort<lwr_fri_msgs::FriJointImpedance> port_fri_joint_impedance;

	InputPort<geometry_msgs::Pose> port_cart_pos_command;
	InputPort<geometry_msgs::Twist> port_cart_vel_command;
	InputPort<geometry_msgs::Wrench> port_cart_wrench_command;
	InputPort<lwr_fri_msgs::CartesianImpedance> port_cart_impedance_command;

	int prop_local_port, m_socket, m_remote_port, m_control_mode;
	std::string joint_names_prefix;
	uint16_t counter, fri_state_last;

	std::string m_mon_mode, m_cmd_mode, m_unknown_mode;
	bool m_init;
	KDL::Jacobian m_jac;

	int m_lost_sample_count, prop_max_lost_samples;
        
        os::TimeService::ticks    m_tstart;       
        os::TimeService::Seconds  m_dt;
};

}//Namespace LWR

#endif//_FRI_COMPONENT_HPP_

