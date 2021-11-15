require("context")
require("geometric")

-- ========================================= PARAMETERS ===================================
maxvel = ctx:createInputChannelScalar("maxvel",0.0)
maxacc = ctx:createInputChannelScalar("maxacc",0.0)

fx_limits = ctx:createInputChannelScalar("force_x_limits")
fy_limits = ctx:createInputChannelScalar("force_y_limits")
fz_limits = ctx:createInputChannelScalar("force_z_limits")
tx_limits = ctx:createInputChannelScalar("torque_x_limits")
ty_limits = ctx:createInputChannelScalar("torque_y_limits")
tz_limits = ctx:createInputChannelScalar("torque_z_limits")

Fx = ctx:createInputChannelScalar("Fx_comp")
Fy = ctx:createInputChannelScalar("Fy_comp")
Fz = ctx:createInputChannelScalar("Fz_comp")
Tx = ctx:createInputChannelScalar("Tx_comp")
Ty = ctx:createInputChannelScalar("Ty_comp")
Tz = ctx:createInputChannelScalar("Tz_comp")

-- ======================================== FRAMES ========================================
-- The task frame corresponds to a frame defined in the robot definition
tf = task_frame

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

Monitor{
    context=ctx,
    name='max_fx_exceeded',
    upper= 0,
    expr= (abs(Fx) - fx_limits),
	actionname='event',
	argument = "e_force_too_high"
}

Monitor{
    context=ctx,
    name='max_fy_exceeded',
    upper= 0,
    expr= (abs(Fy) - fy_limits),
	actionname='portevent',
	argument = "e_force_too_high"
}

Monitor{
    context=ctx,
    name='max_fz_exceeded',
    upper= 0,
    expr= (abs(Fz) - fz_limits),
	actionname='portevent',
	argument = "e_force_too_high"
}

Monitor{
    context=ctx,
    name='max_tx_exceeded',
    upper= 0,
    expr= (abs(Tx) - tx_limits),
	actionname='portevent',
	argument = "e_torque_too_high"
}

Monitor{
    context=ctx,
    name='max_ty_exceeded',
    upper= 0,
    expr= (abs(Ty) - ty_limits),
	actionname='portevent',
	argument = "e_torque_too_high"
}

Monitor{
    context=ctx,
    name='max_tz_exceeded',
    upper= 0,
    expr= (abs(Tz) - tz_limits),
	actionname='portevent',
	argument = "e_torque_too_high"
}

-- ============================== OUTPUT PORTS===================================
ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
ctx:setOutputExpression("z_tf",coord_z(origin(tf)))
roll_tf, pitch_tf, yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)
