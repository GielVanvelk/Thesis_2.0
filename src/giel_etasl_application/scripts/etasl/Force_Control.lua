require("context")
require("geometric")
require("libexpressiongraph_spline")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================
-- Default value is optional
Fz = ctx:createInputChannelScalar("Fz_comp")

maxvel    = ctx:createInputChannelScalar("maxvel")
maxacc    = ctx:createInputChannelScalar("maxacc")
eqradius  = ctx:createInputChannelScalar("eq_r")
delta_x   = ctx:createInputChannelScalar("delta_x")
delta_y   = ctx:createInputChannelScalar("delta_y")
delta_z   = ctx:createInputChannelScalar("delta_z")

force_setpoint     = ctx:createInputChannelScalar("force_setpoint")

-- ======================================== FRAMES ========================================

-- The task frame corresponds to a frame defined in the robot definition
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
endpose   = startpose*translate_x(delta_x)*translate_y(delta_y)*translate_z(delta_z)
endpos    = origin(endpose)
endrot    = rotation(endpose)

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

C_Fz = constant(0.01) -- compliance in z-axis --physical parameter
K_F = constant(0.5) -- setting time for a step; determine with experiments
Constraint{
	context=ctx,
	name= "follow_force_z",
	model =  coord_z(pos_vec),
	meas = -1*C_Fz*(Fz - force_setpoint),
	target = 0,
	K = K_F,
	priority = 2,
	--weight = abs(W_Fx),
};

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
