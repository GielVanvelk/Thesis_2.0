require("context")
require("geometric")
require("libexpressiongraph_spline")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================

maxvel    = ctx:createInputChannelScalar("maxvel" ,0.1)
maxacc    = ctx:createInputChannelScalar("maxacc" ,0.1)

-- ======================================== FRAMES ========================================

tf = task_frame

-- =========================== DEGREE OF ADVANCEMENT =============================================

s = Variable{context = ctx, name ='path_coordinate', vartype = 'feature', initial = 0.0}

-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, tf)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- ========================================== GENERATE ORIENTATION PROFILE ============================

R_end = startrot*rot_z(constant(3.1416/2))

-- eq. axis of rotation for rotation from start to end:w
diff_rot                = cached(getRotVec( inv(startrot)*R_end ))
diff_rot, angle         = utils_ts.normalize( diff_rot )
--
r_inst = angle*s
-- ========================================== GENERATE PROFILES ============================

spl        = CubicSpline(0)
spl:readPoints(rospack_find("giel_etasl_application").."/scripts/deploy/foo.csv"," \t,",0)
spl:setInput(s)

-- =========================== VELOCITY PROFILE ============================================
d_time = constant(0)
mt=constant(1)

A = conditional(time-d_time,constant(1),constant(0))

sa_n = 0
sb_n = 1

sa = constant(sa_n)
sb = constant(sb_n)

s_p_mp  = utils_ts.trap_velprofile( maxvel , maxacc , constant(0.0) , constant(1.0) )

s_p = conditional( time-d_time , s_p_mp , constant(0) )


Constraint{
	context=ctx,
	name = "vel_prof",
	expr = s - s_p*time,
	weight = A*constant(20),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "reaching_vel_max",
	expr = s - maxvel*time,
	weight = A*constant(0.001),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "s_min",
	expr = s,
	target_lower = sa,
	weight = constant(20),
	priority = 2,
	K = constant(4)
};

Constraint{
	context=ctx,
	name = "s_max",
	expr = s,
	target_upper = sb,
	weight = constant(20),
	priority = 2,
	K = constant(4)
};

-- ========================= FOLLOW GENERATED POSE ========================


e_x  = getSplineOutput(spl,0)
e_y  = getSplineOutput(spl,1)
e_z  = getSplineOutput(spl,2)



p_c= startpos + vector(e_x,e_y,e_z)

targetpos = p_c
targetrot = startrot*rotVec(diff_rot, r_inst)

target    = frame(targetrot,targetpos)

Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*tf,
    weight  = constant(10),
    K       = constant(4),
}

-- =================================== MONITOR TO FINISH THE MOTION ========================

err = (sb-s)
Monitor{
        context=ctx,
        name='finish_after_motion',
        lower=0.000,
        actionname='exit',
        expr=err
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
