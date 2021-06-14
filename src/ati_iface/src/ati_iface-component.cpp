#include "ati_iface-component.hpp"
#include <rtt/Component.hpp>

using namespace RTT;

namespace ati
{
/*
    OROCOS component to read ATI F/T Sensor's.
    The scale- and calibration matrices are sensor dependent.
*/
AtiIface::AtiIface( std::string const& _name ) : TaskContext( _name, PreOperational )
    , scale_matrix_( Matrix6d::Zero( ) )
    , prop_scale_matrix_( 36, 0 )
    , calibration_matrix_( Matrix6d::Zero( ) )
    , prop_calibration_matrix_( 36, 0 )
    , prop_lowpass_filter_on_( true )
    , lowpass_filter_on_( true )
    , filter_bandwidth_( 50 )
    , prop_filter_bandwidth_( 50 )
    , filter_order_( 2 )
    , prop_filter_order_( 2 )
    , filter_sample_time_( 0 )
    , filter_buffer_( Matrix6d::Zero( ) )
    , msg_W_ati_( 6, 0 )
    , msg_K_ati_( Matrix6d::Zero( ) )
    , msg_strains_( Vector6d::Zero( ) )
    , msg_strains_offset_( Vector6d::Zero( ) )
    , get_offset_( true )
    , prop_compensate_deadband_( true )
    , compensate_deadband_( true )
    , f_deadband_( 0.01 )
    , prop_f_deadband_( 0.01 )
    , m_deadband_( 0 )
    , prop_m_deadband_( 0 )
{
    // Add properties.
    addProperty( "scale_matrix", prop_scale_matrix_ ).doc( "Scale matrix of ATI F/T sensor. To be multiplied with the calibration matrix." );
    addProperty( "calibration_matrix", prop_calibration_matrix_ ).doc( "The calibration matrix of the corresponding ATI sensor." );
    addProperty( "calibration_matrix", prop_calibration_matrix_ ).doc( "The calibration matrix of the corresponding ATI sensor." );
    addProperty( "lowpass_filter_on", lowpass_filter_on_ ).doc( "Activate low-pass filter." );
    addProperty( "filter_bandwidth", filter_bandwidth_ ).doc( "The bandwith of the ATI force sensor low-pass filter." );
    addProperty( "filter_order", filter_order_ ).doc( "The order (1st or 2nd) of the ATI force sensor low-pass filter." );
    addProperty( "compensate_deadband", compensate_deadband_ ).doc( "Compensate deadband." );
    addProperty( "f_deadband", prop_f_deadband_ ).doc( "Compensate deadband." );
    addProperty( "m_deadband", prop_m_deadband_ ).doc( "Compensate deadband." );

    // Add ports.
    addPort( "in_ai1_port", in_ai1_port_ ).doc( "Analog input port 1 of SoemMasterComponent." );
    addPort( "in_ai2_port", in_ai2_port_ ).doc( "Analog input port 2 of SoemMasterComponent." );
    addPort( "out_W_ati", out_W_ati_ ).doc( "Output wrench of the ATI F/T sensor." );

    // Show messages to the output ports to guarantee real-timeness.
    out_W_ati_.setDataSample( msg_W_ati_ );

    // Resizing of messages.
    msg_ai1_port_.values.resize( 4 );
    msg_ai2_port_.values.resize( 4 );

    log( Info ) << "[" << getName( ) << "] Constructed" << endlog( );
} // End of AtiIface constructor.

/*
    Configuration hook of the OROCOS component.
*/
bool AtiIface::configureHook( ) {
    // Check if scale- and calibration matrices are assigned.
    if ( Matrix6d( prop_scale_matrix_.data( ) ).norm( ) == 0 ) {
        log( Error ) << "[" <<  getName( ) << "] There is no scale matrix assigned." << endlog( );

        return false;
    }

    if ( Matrix6d( prop_calibration_matrix_.data( ) ).norm( ) == 0 ) {
        log( Error ) << "[" <<  getName( ) << "] There is no calibration matrix assigned." << endlog( );

        return false;
    }

    // Set verified properties.
    scale_matrix_ = Matrix6d ( prop_scale_matrix_.data( ) );
    calibration_matrix_ = Matrix6d ( prop_calibration_matrix_.data( ) );
    filter_bandwidth_ = prop_filter_bandwidth_;
    if ( prop_filter_order_ == 1 || prop_filter_order_ == 2 ) {
        filter_order_ = prop_filter_order_;
    }
    else {
        log( Warning ) << "[" << getName( ) << "] lowPassFilter: Wrong order." << endlog( );
    }
    f_deadband_ = prop_f_deadband_;
    m_deadband_ = prop_m_deadband_;

    // The actual calibration matrix after scaling.
    msg_K_ati_ = scale_matrix_ * calibration_matrix_;

    // The low-pass filter sample time.
    filter_sample_time_ = getPeriod( );

    log( Info ) << "[" << getName( ) << "] Configured" << endlog( );

    return true;
} // End of configureHook.

/*
    Start hook of the OROCOS component.
*/
bool AtiIface::startHook( ) {
    // set initial message to 0 and write to output port.
    std::fill( msg_W_ati_.begin( ), msg_W_ati_.end( ), 0 );
    out_W_ati_.setDataSample( msg_W_ati_ );

    // Turn low-pass filter on/off.
    lowpass_filter_on_ = prop_lowpass_filter_on_;

    // Compensate deadband.
    compensate_deadband_ = prop_compensate_deadband_;

    // Get strain offset after start.
    get_offset_ = true;

    // Reset input ports.
    in_ai1_port_.clear( );
    in_ai2_port_.clear( );

    // Reset strain message.
    msg_strains_ = Vector6d::Zero( );

    // Reset low-pass filter buffer.
    filter_buffer_ = Matrix6d::Zero( );

    log( Info ) << "[" << getName( ) << "] Started" << endlog( );

    return true;
} // End of startHook.

/*
    Update hook of the OROCOS component.
*/
void AtiIface::updateHook( ) {
    // Read analog input ports.
    in_ai1_port_.read( msg_ai1_port_ );
    in_ai2_port_.read( msg_ai2_port_ );

    // Get strain offset after start.
    if ( get_offset_ == true ) {
        for( int i = 0; i < 4; i++ ) {
            msg_strains_offset_[ i ] = msg_ai1_port_.values[ i ];
        }
        for( int i = 0; i < 2; i++ ) {
            msg_strains_offset_[ i + 4 ] = msg_ai2_port_.values[ i ];
        }

        get_offset_ = false;
    }

    // Create strain vector from input port data.
    for( int i = 0; i < 4; i++ ) {
        msg_strains_[ i ] = msg_ai1_port_.values[ i ];
    }
    for( int i = 0; i < 2; i++ ) {
        msg_strains_[ i + 4 ] = msg_ai2_port_.values[ i ];
    }

    // Calculate wrench from strain data.
    Vector6d W_ati_ = msg_K_ati_ * ( msg_strains_ - msg_strains_offset_ );

    // Filter the wrench data.
    if ( lowpass_filter_on_ ) {
        W_ati_ = lowPassFilter( W_ati_, filter_order_, 2 * PI * filter_bandwidth_, filter_sample_time_, filter_buffer_ );
    }

    // Compensate deadband.
    if ( compensate_deadband_ ) {
    W_ati_ = deadBandEigen( W_ati_, fabs( f_deadband_ ), m_deadband_ );
    }

    // Write wrench to output port.
    for( int i = 0; i < 6; i++ ) {
        msg_W_ati_[ i ] = W_ati_[ i ];
    }

    out_W_ati_.write( msg_W_ati_ );
} // End of updateHook.

/*
    Stop hook of the OROCOS component.
*/
void AtiIface::stopHook( ) {
    // set final message to 0 and write to output port.
    std::fill( msg_W_ati_.begin( ), msg_W_ati_.end( ), 0 );
    out_W_ati_.setDataSample( msg_W_ati_ );

    log( Info ) << "[" << getName( ) << "] Stopped" << endlog( );
} // End of stopHook.

/*
    Cleanup hook of the OROCOS component.
*/
void AtiIface::cleanupHook( ) {
    log( Info ) << "[" << getName( ) << "] Cleaned up" << endlog( );
} // End of cleanupHook.

/*
    First or second order low-pass filter.
*/
Vector6d AtiIface::lowPassFilter( const Vector6d _W_ati_unfiltered, int order, double bw, double ts, Matrix6d &buffer ) {
    // Tustin discretization rule.
    double b_0, b_1, b_2, a_0, a_1, a_2;
    if ( order == 1 ) {
        b_0 = bw * ts;
        b_1 = bw * ts;
        b_2 = 0;
        a_0 = ts * bw + 2;
        a_1 = ts * bw - 2;
        a_2 = 0;
    } else if ( order == 2 ) {
        b_0 = bw * bw * ts * ts;
        b_1 = 2 * bw * bw * ts * ts;
        b_2 = bw * bw * ts * ts;
        a_0 = 4 * ts * bw + bw * bw * ts * ts + 4;
        a_1 = -8 + 2 * bw * bw * ts * ts;
        a_2 = -4 * ts * bw + bw * bw * ts * ts + 4;
    } else {
        log( Warning ) << "[" << getName( ) << "] lowPassFilter: Wrong order." << endlog( );
    return _W_ati_unfiltered;
    }

    Vector6d _W_ati_filtered;
    for ( int i = 0; i < 6; i++ ) {
        buffer( 2, i ) = buffer( 1, i );
        buffer( 1, i ) = buffer( 0, i );
        buffer( 0, i ) = _W_ati_unfiltered( i );
        buffer( 5, i ) = buffer( 4, i );
        buffer( 4, i ) = buffer( 3, i );
        buffer( 3, i ) = ( b_0 * buffer( 0, i ) + b_1 * buffer( 1, i ) + b_2 * buffer( 2, i ) - a_1 * buffer( 4, i ) - a_2 * buffer( 5, i ) ) / a_0;
        _W_ati_filtered( i ) = buffer( 3, i );
    }
    return _W_ati_filtered;
} // lowPassFilter

/*
    Deadband around value for EigenVector.
*/
Vector6d AtiIface::deadBandEigen( const Vector6d _W_ati, double f_deadband_, double m_deadband_ ) {
  Vector6d _W_ati_deadband;
    // Deadband on force.
    for ( int i = 0; i < 3; i++ ) {
        if ( _W_ati( i ) > f_deadband_ ) {
            _W_ati_deadband( i ) = ( _W_ati( i ) - f_deadband_ );
        } else if ( _W_ati( i ) < -1 * f_deadband_ ) {
            _W_ati_deadband( i ) = ( _W_ati( i ) + f_deadband_ );
        } else {
            _W_ati_deadband( i ) = 0;
        }
    }

    // Deadband on torque (NOT).
    for ( int i = 3; i < 6; i++ )
        _W_ati_deadband( i ) = _W_ati( i );
    return _W_ati_deadband;
}
}   // End of namespace ati.

ORO_CREATE_COMPONENT( ati::AtiIface )

/*
    @author:        Jef De Smet
    @email:         jef.desmet@kuleuven.be
    @affiliation:   KU Leuven, Department of Mechanical Engineering, Leuven, Belgium
*/
