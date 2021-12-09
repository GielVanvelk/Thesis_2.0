#include "giel_component.hpp"
#include <iostream>

namespace gcomp
{
	// Detect zero values in pose data
	bool detect_zero_pose_data(std::vector < double > pose_data)
	{
		// if all pose values are 0 it means no pose data has been received
		if (pose_data[0] == 0 && pose_data[1] == 0 && pose_data[2] == 0 && pose_data[3] == 0 && pose_data[4] == 0 && pose_data[5] == 0){
			return true;
		}
		else{
			return false;
		}
	}

	// Force transformation from sensor frame to tool frame
	std::vector < double > sensor_to_tool_frame(std::vector < double > force_data_sensor_frame, std::vector < double > transformation_params)
	{
		// Create a new vector
		std::vector < double > force_data_tool_frame;
		force_data_tool_frame.push_back(0.0);
		force_data_tool_frame.push_back(0.0);
		force_data_tool_frame.push_back(0.0);
		force_data_tool_frame.push_back(0.0);
		force_data_tool_frame.push_back(0.0);
		force_data_tool_frame.push_back(0.0);
		// The tool frame has the same orientation as the sensor frame
		force_data_tool_frame[0] = force_data_sensor_frame[0];
		force_data_tool_frame[1] = force_data_sensor_frame[1];
		force_data_tool_frame[2] = force_data_sensor_frame[2];
		// The tool frame is translated along the z axis of the sensor frame.
		force_data_tool_frame[3] = force_data_sensor_frame[3] - transformation_params[2]*force_data_sensor_frame[1];
		force_data_tool_frame[4] = force_data_sensor_frame[4] - transformation_params[2]*force_data_sensor_frame[0];
		force_data_tool_frame[5] = force_data_sensor_frame[5];
		// Return the force data in the tool frame
		return force_data_tool_frame;
	}

	// Determine sensor compensation parameters
	std::vector < double > sensorCompensationParams(std::vector < double > force_data, int sample_size)
	{
		std::vector<double> ss_params;
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params = autoSensorCompensationParams(force_data, sample_size);
		return ss_params;
	}

	// Calculate the parameters to compensate for the steady state error
	std::vector < double > autoSensorCompensationParams(std::vector < double > force_data, int sample_size)
	{
		std::vector<double> x_force_samples;
		std::vector<double> y_force_samples;
		std::vector<double> z_force_samples;
		std::vector<double> x_moment_samples;
		std::vector<double> y_moment_samples;
		std::vector<double> z_moment_samples;
		std::vector<double> ss_comp;
		ss_comp.push_back(0.0);
		ss_comp.push_back(0.0);
		ss_comp.push_back(0.0);
		ss_comp.push_back(0.0);
		ss_comp.push_back(0.0);
		ss_comp.push_back(0.0);
		for (int i = 0; i < sample_size; i++){
			x_force_samples.push_back( force_data[0] );
			y_force_samples.push_back( force_data[1] );
			z_force_samples.push_back( force_data[2] );
			x_moment_samples.push_back( force_data[3] );
			y_moment_samples.push_back( force_data[4] );
			z_moment_samples.push_back( force_data[5] );
		}
		ss_comp[0] = average_vector(x_force_samples);
		ss_comp[1] = average_vector(y_force_samples);
		ss_comp[2] = average_vector(z_force_samples);
		ss_comp[3] = average_vector(x_moment_samples);
		ss_comp[4] = average_vector(y_moment_samples);
		ss_comp[5] = average_vector(z_moment_samples);
		return ss_comp;
	}

	//Function to calculate the average of an array
	double average_vector ( std::vector < double > v )
	{
		double sum = 0.0;
		double average = 0.0;
		int n = v.size();
		for ( int i=0; i < n; i++){
			sum += v[i];
		}
		average = sum/n;
		return average;
	}

	// Function for compensating for the steady state error on the force sensor
	std::vector < double > sensorCompensation(std::vector < double > force_data, std::vector < double > compensation_params)
	{
		std::vector<double> force_data_compensated;
		force_data_compensated.push_back( 0.0 );
		force_data_compensated.push_back( 0.0 );
		force_data_compensated.push_back( 0.0 );
		force_data_compensated.push_back( 0.0 );
		force_data_compensated.push_back( 0.0 );
		force_data_compensated.push_back( 0.0 );
		force_data_compensated[0] = force_data[0] - compensation_params[0];
		force_data_compensated[1] = force_data[1] - compensation_params[1];
		force_data_compensated[2] = force_data[2] - compensation_params[2];
		force_data_compensated[3] = force_data[3] - compensation_params[3];
		force_data_compensated[4] = force_data[4] - compensation_params[4];
		force_data_compensated[5] = force_data[5] - compensation_params[5];
		return force_data_compensated;
	}

	// Function to reduce the zero noice
	std::vector < double > reduceZeroNoice(std::vector < double > data, double cutoff)
	{
		double data1 = data[0];
		double data2 = data[1];
		double data3 = data[2];
		double data4 = data[3];
		double data5 = data[4];
		double data6 = data[5];
		std::vector<double> new_data;
		new_data.push_back(0.0);
		new_data.push_back(0.0);
		new_data.push_back(0.0);
		new_data.push_back(0.0);
		new_data.push_back(0.0);
		new_data.push_back(0.0);
		double cutoff_neg = -1 * cutoff;
		if (data1 > cutoff_neg && data1 < cutoff) {
			data1 = 0;
		}
		if (data2 > cutoff_neg && data2 < cutoff) {
			data2 = 0;
		}
		if (data3 > cutoff_neg && data3 < cutoff) {
			data3 = 0;
		}
		if (data4 > cutoff_neg && data4 < cutoff) {
			data4 = 0;
		}
		if (data5 > cutoff_neg && data5 < cutoff) {
			data5 = 0;
		}
		if (data6 > cutoff_neg && data6 < cutoff) {
			data6 = 0;
		}
		new_data[0] = data1;
		new_data[1] = data2;
		new_data[2] = data3;
		new_data[3] = data4;
		new_data[4] = data5;
		new_data[5] = data6;
		return new_data;
	}

	//Function for calculating the stiffness
	double calc_stiffness ( std::vector < double > force, std::vector < double > pose )
	{
		int number_of_data_samples = force.size();
		double force_average = average_vector(force);
		double pose_average = average_vector(pose);
		double xy_sum = 0;
		double x_diff_kw_sum = 0;

		for (int i = 0; i < number_of_data_samples; i++){
			double x_diff = (pose[i] - pose_average);
			double y_diff = (force[i] - force_average);

			xy_sum = xy_sum + (y_diff * x_diff);
			x_diff_kw_sum = x_diff_kw_sum + (x_diff * x_diff);
		}
		double stiffness = xy_sum/x_diff_kw_sum;
		return stiffness;
	}

	//Function for sensor filtering
	double sensor_filtering(std::vector < double > data, int size)
	{
		std::vector<double> forceZ_data_compensated;

		if (forceZ_data_compensated.size() <size){
			forceZ_data_compensated.push_back(data[2]);
		}
		else{
			for (int i = 0; i < size-1; i++) {
  				forceZ_data_compensated[i]= forceZ_data_compensated[i+1];
				forceZ_data_compensated.push_back(data[2]);
			}
		}
		return average_vector(forceZ_data_compensated);
	}

}
