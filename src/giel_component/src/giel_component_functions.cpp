#include "giel_component.hpp"

namespace gcomp
{

	std::vector < double > sensorCompensationParams(std::vector < double > force_data, int comp_type ,int sample_size)
	{

		std::vector<double> ss_params;
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);
		ss_params.push_back(0.0);

		if (comp_type == 1){
			ss_params = autoSensorCompensationParams(force_data, sample_size);
		}
		else{
			// compensation for steady state error (according to normal direction)
			ss_params[0] = 3.85; //force gets negative when pushing against the positive z axis (tf), (pushing against the probe)
			ss_params[1] = 13.35; //force gets negative when pushing along positive y axis
			ss_params[2] = -9.99; //force gets positive when pushing against the positive x axis (tf),
			ss_params[3] = 0.0;
			ss_params[4] = 0.0;
			ss_params[5] = 0.0;
		}

		return ss_params;
	}


	// Function for compensating the force data (gravity)
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

		// return the compensated force signal
		return force_data_compensated;
	}

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

		for (int i = 0; i < sample_size; i++)
		{
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

		//bool autocomp = false;
		//if (force_data[2] >= 10 || force_data[2] <= -10 || force_data[0] >= 3 ){
		//	autocomp = true;
		//}

		//return autocomp;
	}

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

	//Function for average
	double average_vector ( std::vector < double > v )
	{
		double sum = 0.0;
		double average = 0.0;
		int n = v.size();
		for ( int i=0; i < n; i++)
		{
			sum += v[i];
		}
		average = sum/n;
		return average;
	}

	//Function for average
	double calc_stiffness ( std::vector < double > force, std::vector < double > pose )
	{
		int number_of_data_samples = force.size();

		double Fz1 = force[0];
		double Fz2 = force[0];
		double Fdiff = Fz2 - Fz1; //[N]

		double Pz1 = pose[number_of_data_samples -1];
		double Pz2 = pose[number_of_data_samples -1];
		double Pdiff = Pz2 - Pz1; //[m]

		double stiffness = Fdiff / Pdiff; //[N/m]

		return stiffness;
	}




	//Function to save data to a bag-file_name
	//void safe_to_bag() {

		//rosbag::Bag bag("test.bag", rosbag::bagmode::Write);
		//rosbag::Bag bag;
		//bag.open("test.bag", rosbag::bagmode::Write);

		//std_msgs::Int32 i;
		//i.data = 42;
		//bag.write("numbers", ros::Time::now(), i);
		//bag.close();

		//rosbag::Bag bag;
		//bag.open("test.bag", rosbag::bagmode::Write);

		//std_msgs::String str;
		//str.data = std::string("foo");
		//std_msgs::Int32 i;
	    //i.data = 42;

		//bag.write("chatter", ros::Time::now(), str);
		//bag.write("numbers", ros::Time::now(), i);

		//bag.close();
	//}

}
