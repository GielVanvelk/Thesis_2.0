#ifndef ATI_IFACE_COMPONENT_HPP
#define ATI_IFACE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <Eigen/Geometry>

#define PI 3.14159265

namespace ati
{
    /*
        OROCOS component to read ATI F/T Sensor's.
        The scale- and calibration matrices are sensor dependent.
    */
// Define vector and matrix types.
typedef Eigen::Matrix < double, 6, 1 > Vector6d;
typedef Eigen::Matrix < double, 6, 6, Eigen::RowMajor > Matrix6d;

class AtiIface : public RTT::TaskContext {
    public:

        AtiIface( std::string const& _name );
        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();

    protected:

        // Properties
        std::vector < double > prop_scale_matrix_;
        std::vector < double > prop_calibration_matrix_;
        bool prop_lowpass_filter_on_;
        double prop_filter_bandwidth_;
        int prop_filter_order_;
        bool prop_compensate_deadband_;
        double prop_f_deadband_;
        double prop_m_deadband_;

        // Ports
        RTT::InputPort < soem_beckhoff_drivers::AnalogMsg > in_ai1_port_;
        RTT::InputPort < soem_beckhoff_drivers::AnalogMsg > in_ai2_port_;
        RTT::OutputPort < std::vector < double > > out_W_ati_; 

    private:
        // Members.
        Vector6d lowPassFilter( const Vector6d _W_ati_unfiltered, int order, double bw, double ts, Matrix6d &fbuffer ); // First or second order low-pass filter.
        Vector6d deadBandEigen( const Vector6d _W_ati, double f_deadband_, double m_deadband_ );

        // Verified properties.
        Matrix6d scale_matrix_;
        Matrix6d calibration_matrix_;
        bool lowpass_filter_on_;
        double filter_bandwidth_;
        int filter_order_;
        double filter_sample_time_;
        Matrix6d filter_buffer_;
        bool compensate_deadband_;
        double f_deadband_;
        double m_deadband_;

        // Messages
        soem_beckhoff_drivers::AnalogMsg msg_ai1_port_;
        soem_beckhoff_drivers::AnalogMsg msg_ai2_port_;
        std::vector < double > msg_W_ati_;
        Matrix6d msg_K_ati_;
        Vector6d msg_strains_;
        Vector6d msg_strains_offset_;

        // Flags
        bool get_offset_;

        // Low-pass filter variables.
        double time_constant_; // [s]
        std::vector < double > input_buffer_; // Buffer for digital filter.
        std::vector < double > output_buffer_; // Buffer for digital filter.

};
}   // End of namespace ati.

#endif // End of ATI_IFACE_COMPONENT

/*
    @author:        Jef De Smet
    @email:         jef.desmet@kuleuven.be
    @affiliation:   KU Leuven, Department of Mechanical Engineering, Leuven, Belgium
*/
