require("context")
require("geometric")
require("libexpressiongraph_spline")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================
Fx_raw = ctx:createInputChannelScalar("Fz")
Fy_raw = ctx:createInputChannelScalar("Fy")
Fz_raw = ctx:createInputChannelScalar("Fx")

--Fx_raw = ctx:createInputChannelScalar("Fz_comp")
--Fy_raw = ctx:createInputChannelScalar("Fz_comp")
--Fz_raw = ctx:createInputChannelScalar("Fz_comp")

maxvel            = ctx:createInputChannelScalar("maxvel" ,0)
maxacc            = ctx:createInputChannelScalar("maxacc" ,0)
endpose           = ctx:createInputChannelFrame("endpose")
eqradius          = ctx:createInputChannelScalar("eq_r"   ,0)
force_set         = ctx:createInputChannelScalar("force_set" ,0)


th_f=constant(0.1)
th_t=constant(0.03)
Fx  = utils_ts.dead_zone(Fx_raw,th_f)
Fy  = utils_ts.dead_zone(Fy_raw,th_f)
Fz  = utils_ts.dead_zone(Fz_raw,th_f)
Tx = utils_ts.dead_zone(Tx_raw,th_t)
Ty = utils_ts.dead_zone(Ty_raw,th_t)
Tz = utils_ts.dead_zone(Tz_raw,th_t)



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

--C_Fz = constant(0.0025) -- compliance in z-axis
C_Fz = constant(0.01) -- compliance in z-axis
K_F = constant(4)
Constraint{
	context=ctx,
	name="follow_force_z",
	model = coord_z(pos_vec),
	meas = -C_Fz*Fz,
	target = -C_Fz* force_set,
	K = K_F,
	priority = 2,
	--weight = abs(W_Fx),
};

-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(0.1)
}

Monitor {
    context = ctx,
    name = "force_monitor_x",
    expr = Fx, -- When expr exceeds the lower or upper bound, the event is triggered (only once).
    lower = -100,
	upper = 100,
    actionname = "portevent",
    argument = "force_error_x" -- sent flag to state machine
}

Monitor {
    context = ctx,
    name = "force_monitor_y",
    expr = Fy, -- When expr exceeds the lower or upper bound, the event is triggered (only once).
	lower = -100,
	upper = 100,
    actionname = "portevent",
    argument = "force_error_y" -- sent flag to state machine
}

Monitor {
    context = ctx,
    name = "force_monitor_z",
    expr = Fz, -- When expr exceeds the lower or upper bound, the event is triggered (only once).
	lower = -100,
	upper = 100,
    actionname = "portevent",
    argument = "force_error_z" -- sent flag to state machine
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
