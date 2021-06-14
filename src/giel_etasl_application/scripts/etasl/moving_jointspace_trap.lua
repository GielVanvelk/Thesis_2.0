-- Example of trapezoidal motionprofile, applied to a joint space movement:
-- Requires the following etal variables to be set:
--     maxvel
--     maxacc
--     end_1 ... end_n (jval at the end of the motion)


require("context")
require("geometric")

-- ========================================= PARAMETERS ===================================
maxvel = ctx:createInputChannelScalar("maxvel",0.5)
maxacc = ctx:createInputChannelScalar("maxacc",0.5)

-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value

-- for i=1,#robot_joints do
--     current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
--     mp:addOutput( initial_value(time, current_jnt[i]), ctx:createInputChannelScalar("end_j"..i), maxvel, maxacc)
-- end

-- The following creates a trapezoidal velocity profile from the initial value of each angle, towards the target angle. It checks whether the joint is continuous or bounded,
-- and if it is continuous it takes the shortest path towards the angle. This makes the skill generic to any type of robot (e.g. the Kinova).
-- The old version used the above commented method, which is the one that is explained in the etasl tutorial.
for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    local theta_init = initial_value(time, current_jnt[i])
    local theta_final_raw = ctx:createInputChannelScalar("end_j"..i)
    local difference_theta = cached(acos(cos(theta_init)*cos(theta_final_raw)+sin(theta_init)*sin(theta_final_raw))) --Shortest angle between two unit vectors (basic formula: 'cos(alpha) = dot(a,b)'. where a and b are two unit vectors)
    local error_difference_theta = cached(acos(cos(theta_init + difference_theta)*cos(theta_final_raw)+sin(theta_init + difference_theta)*sin(theta_final_raw))) --Shortest angle computation also. If the sign is correct, it should be zero
    local delta_theta = cached(conditional(error_difference_theta - constant(1e-5) ,constant(-1)*difference_theta,difference_theta)) --determines the proper sign to rotate the initial angle

    local is_continuous = ctx:createInputChannelScalar("continuous_j"..i,0)--TODO: In the next release we will be able to obtain this directly from the urdf
    local final_angle = cached(conditional(constant(-1)*abs(is_continuous),theta_final_raw,theta_init + delta_theta)) -- Only 0 is interpreted as bounded
    mp:addOutput( theta_init, make_constant(final_angle) , maxvel, maxacc)
end

duration = get_duration(mp)
-- print(duration:value())

-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
for i=1,#robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt[i] - tgt[i] ,
        priority = 2,
        K=4
    };
end

-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-duration
}
