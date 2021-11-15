require("context")
require("geometric")
require("libexpressiongraph_spline")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================
Fx= ctx:createInputChannelScalar("Fz_comp")
Fy= ctx:createInputChannelScalar("Fy_comp")
Fz= ctx:createInputChannelScalar("Fx_comp")

maxvel            = ctx:createInputChannelScalar("maxvel" ,0)
maxacc            = ctx:createInputChannelScalar("maxacc" ,0)
endpose           = ctx:createInputChannelFrame("endpose")
eqradius          = ctx:createInputChannelScalar("eq_r"   ,0)
force_set         = ctx:createInputChannelScalar("force_set" ,0)
stiffness         = ctx:createInputChannelScalar("stiffness_val" ,0)

fx_limits = ctx:createInputChannelScalar("force_x_limits")
fy_limits = ctx:createInputChannelScalar("force_y_limits")
fz_limits = ctx:createInputChannelScalar("force_z_limits")
tx_limits = ctx:createInputChannelScalar("torque_x_limits")
ty_limits = ctx:createInputChannelScalar("torque_y_limits")
tz_limits = ctx:createInputChannelScalar("torque_z_limits")

-- ======================================== FRAMES ========================================

tf = task_frame

-- =========================== DEGREE OF ADVANCEMENT =============================================
s = Variable{
  context = ctx,
  name ='path_coordinate',
  vartype = 'feature',
  initial = 0.0
}

-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, tf)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================
endpos    = origin(endpose)
endrot    = rotation(endpose)

z_start = coord_z(startpos)
z_end = coord_z(endpos)

endpos_adapted = endpos - vector(0,0,z_end) + vector(0,0,z_start)
endpos = endpos_adapted

-- ========================================== GENERATE ORIENTATION PROFILE ============================

R_end = startrot*rot_z(constant(3.1416/2))

-- eq. axis of rotation for rotation from start to end:w
diff_rot                = cached(getRotVec( inv(startrot)*R_end ))
diff_rot, angle         = utils_ts.normalize( diff_rot )
--
r_inst = angle*s


-- =========================== VELOCITY PROFILE ============================================

-- compute distances for displacements and rotations:
diff                    = cached(endpos-startpos)
diff, distance          = utils_ts.normalize( diff )

diff_rot                = cached(  getRotVec( inv(startrot)*endrot )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = utils_ts.normalize( diff_rot )


-- plan trapezoidal motion profile in function of time:
mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
mp:addOutput(constant(0), distance, maxvel, maxacc)
mp:addOutput(constant(0), angle*eqradius, maxvel, maxacc)
d  = get_output_profile(mp,0)            -- progression in distance
r  = get_output_profile(mp,1)/eqradius   -- progression in distance_rot (i.e. rot*eqradius)

-- =========================== TARGET POSE ============================================

targetpos = startpos + diff*d
targetrot = startrot*rotVec(diff_rot,r)

target    = frame(targetrot,targetpos)

task_frame_i = inv(make_constant(tf))*tf
pos_vec = origin(task_frame_i)

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*tf,
    K       = 3,
    weight  = 1,
    priority= 2
}

--print(constant(1/stiffness))
C_Fz = constant(0.5) -- compliance in z-axis --physical parameter
K_F = constant(0.1) -- setting time for a step; determine with experiments


Constraint{
	context=ctx,
	name= "follow_force_z",
	model =  coord_z(origin(tf)),
	meas = 1*(C_Fz)*(Fz - force_set),
	target = 0,
	K = K_F,
	priority = 2,
};

-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(0.1)
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
tf_origin = origin(tf)

ctx:setOutputExpression("x_tf",coord_x(tf_origin))
ctx:setOutputExpression("y_tf",coord_y(tf_origin))
ctx:setOutputExpression("z_tf",coord_z(tf_origin))
roll_tf, pitch_tf, yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)

ctx:setOutputExpression("s",s)
