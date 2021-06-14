#ifndef GIEL_COMPONENT_HPP
#define GIEL_COMPONENT_HPP

#include <geometry_msgs/PoseStamped.h>
#include <rtt/RTT.hpp>
#include <string>
#include <Eigen/Geometry>

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

        // Properties

        // INPUT-Ports
        RTT::InputPort < std::vector < double > >       in_force_data_;

        // OUTPUT-Ports
        RTT::OutputPort < bool >                        out_force_too_low_;
        RTT::OutputPort < bool >                        out_force_too_high_;

    private:
        // Messages
        std::vector < double >      msg_force_data_;
        bool      msg_force_too_high_ ;
        bool      msg_force_too_low_ ;
};

} // End of namespace usconnector.

#endif
