#include "giel_component.hpp"
#include <rtt/Component.hpp>
#include <unistd.h>

#include <iomanip>
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <vector>

#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>



using namespace RTT;
using namespace Eigen;

namespace gcomp
{
/*
    Constructor of the OROCOS component.
*/
GielComponent::GielComponent( std::string const& _name ) : TaskContext( _name, PreOperational )
    , msg_force_too_high_(false)
    , msg_force_too_low_(false)
    , msg_force_data_( 6, 0 )
    , msg_force_(0.0)
    , msg_force_pre_(0.0)
{

    // Add ports. (addEventPort() for a port that wakes up the activity)
    addPort("in_force_data", in_force_data_).doc( "The force data from the force sensor");

    addPort("out_force_too_low", out_force_too_low_ ).doc( "Message that the applied force needs to be increased.");
    addPort("out_force_too_high", out_force_too_high_ ).doc( "Message that the applied force needs to be decreased.");
    addPort("out_force", out_force_ ).doc( "force.");

    // Add operations.

    // Show messages to the output ports to guarantee real-timeness.
    out_force_too_low_.setDataSample( msg_force_too_low_);
    out_force_too_high_.setDataSample( msg_force_too_high_);
    out_force_.setDataSample( msg_force_);

    // Message if completed
    log( Info ) << "[" << getName( ) << "] Constructed" << endlog( );

}

/**
 * This function is for the configuration code.
 * Return false to abort configuration.
 */
bool GielComponent::configureHook( )
{
    if ( msg_force_too_low_ )
        {
            msg_force_too_low_ = false;
        }
    if ( msg_force_too_high_ )
        {
            msg_force_too_high_ = false;
        }

    log( Info ) << "[" << getName( ) << "] Configured" << endlog( );
    return true;
}

/**
 * This function is for the application's start up code.
 * Return false to abort start up.
 */
bool GielComponent::startHook( )
{
    // Set output ports and initial msgs.
    msg_force_too_low_ = false ;
    out_force_too_low_.write( msg_force_too_low_ );

    msg_force_too_high_= false ;
    out_force_too_high_.write(  msg_force_too_high_);

    msg_force_ = 0.0;
    out_force_.write( msg_force_ );

    // Reset input ports.
    in_force_data_.clear();

    // Reset flags.

    log( Info ) << "[" << getName( ) << "] Started" << endlog( );
    return true;
}

/*
    Update hook of the OROCOS component.
*/
void GielComponent::updateHook( )
{
    // Message if executing the updateHook
    // std::cout << "giel_component executes updateHook !" <<std::endl;

    if(in_force_data_.read( msg_force_data_) == RTT::NewData){

    in_force_data_.read( msg_force_data_);

    double fx = msg_force_data_ [0];
    //cout << fx;
    double fy = msg_force_data_ [1];
    //cout << fy;
    double fz = msg_force_data_ [2];
    //cout << fz;
    msg_force_ = sqrt(fx*fx + fy*fy + fz*fz);

    //if (msg_force_ == 0) {
    //  msg_force_ = msg_force_pre_;
    //}
    msg_force_pre_ = msg_force_;
    //cout << msg_force_;
    out_force_.write(msg_force_);

    //double fx = msg_force_data_;

    if (msg_force_ >= 6.00) {
      //cout << "been here";
      msg_force_too_low_ = false;
      out_force_too_low_.write(msg_force_too_low_);

      msg_force_too_high_ = true;
      out_force_too_high_.write(msg_force_too_high_);
    }
    else if (msg_force_  <= 3.00 ) {
      msg_force_too_low_ = true;
      out_force_too_low_.write(msg_force_too_low_);

      msg_force_too_high_ = false;
      out_force_too_high_.write(msg_force_too_high_);
    }
    else {
      msg_force_too_low_ = false;
      out_force_too_low_.write(msg_force_too_low_);

      msg_force_too_high_ = false;
      out_force_too_high_.write(msg_force_too_high_);
    }
  }

}

/*
    Stop hook of the OROCOS component.
*/
void GielComponent::stopHook( )
{
    msg_force_too_low_ = false ;
    out_force_too_low_.write(false);

    msg_force_too_high_= false ;
    out_force_too_high_.write(false);

    msg_force_ = 0.0;
    out_force_.write(0.0);

    log( Info ) << "[" << getName( ) << "] Stopped" << endlog( );
}

/*
    Cleanup hook of the OROCOS component.
*/
void GielComponent::cleanupHook( )
{
    log( Info ) << "[" << getName( ) << "] Cleaned up" << endlog( );
}

} // End of namespace Usconnector.

ORO_CREATE_COMPONENT( gcomp::GielComponent )
