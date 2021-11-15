-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Main file to combine motions by learning or modeling information
-- KU Leuven 2020
-- ==============================================================================

require("context")
require("geometric")
require("motion_model")
utils_ts = require("utils_ts")

-- ========================================= PARAMETERS ===================================

maxvel    = ctx:createInputChannelScalar("maxvel" ,0.1)
maxacc    = ctx:createInputChannelScalar("maxacc" ,0.1)

c_tgt_x	  = ctx:createInputChannelScalar("c_tgt_x",0.4)
c_tgt_y	  = ctx:createInputChannelScalar("c_tgt_y",0.0)
c_tgt_z	  = ctx:createInputChannelScalar("c_tgt_z",0.1)

motion_model_file = rospack_find("giel_etasl_application").."/scripts/etasl/motion_models/solenoid_case.json"
-- ======================================== FRAMES ========================================

tf = task_frame

-- =========================== DEGREE OF ADVANCEMENT =============================================

s = Variable{context = ctx, name ='path_coordinate', vartype = 'feature', initial = 0.0}


-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, tf)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- ========================================== GENERATE ORIENTATION PROFILE ============================

R_end = startrot

diff_rot                = cached(getRotVec( inv(startrot)*R_end ))
diff_rot, angle         = utils_ts.normalize( diff_rot )

r_inst = angle*s
-- ========================================== GENERATE POSITION PROFILE ============================

str=read_to_string(motion_model_file)
obj = read_json( str)
m = MotionModel:construct( obj, s )
fv_mm,fv_names_mm=m:create_fv_and_constr(ctx,"motion_model",0,0.1)

e1_end=m:dof(1):expression_value(fv_mm,1)
e2_end=m:dof(2):expression_value(fv_mm,1)
e3_end=m:dof(3):expression_value(fv_mm,1)

f_end = startpos + vector(e1_end,e2_end,e3_end)

Constraint{
	context=ctx,
	name = "end point_x",
	expr = coord_x(f_end) - c_tgt_x,
	weight = constant(5),
	priority = 1,
	K = constant(20)
};
Constraint{
	context=ctx,
	name = "end point_y",
	expr = coord_y(f_end) - c_tgt_y,
	weight = constant(5),
	priority = 1,
	K = constant(20)
};
Constraint{
	context=ctx,
	name = "end point_z",
	expr = coord_z(f_end) - c_tgt_z,
	weight = constant(5),
	priority = 1,
	K = constant(20)
};

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
	weight = constant(8),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "reaching_vel_max",
	expr = s - maxvel*time,
	weight = A*constant(1),
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

e_x  = cached(m:dof(1):expression(fv_mm))
e_y  = cached(m:dof(2):expression(fv_mm))
e_z  = cached(m:dof(3):expression(fv_mm))


p_c= startpos + vector(e_x,e_y,e_z)

targetpos = p_c
targetrot = startrot*rotVec(diff_rot, r_inst)

target    = frame(targetrot,targetpos)

Constraint{
	context = ctx,
	name    = "follow_path",
	expr    = inv(target)*tf,
	weight  = A*constant(4),
	K       = constant(4),
}

-- =================================== MONITOR TO FINISH THE MOTION ========================

err = (sb-s)


Monitor{
	context=ctx,
	name='finish_after_motion',
	lower=0.0005,
	actionname='exit',
	expr=err
}


-- ============================== OUTPUT PORTS ===================================
tf_origin = origin(tf)

ctx:setOutputExpression("x_tf",coord_x(tf_origin))
ctx:setOutputExpression("y_tf",coord_y(tf_origin))
ctx:setOutputExpression("z_tf",coord_z(tf_origin))
roll_tf, pitch_tf, yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)

ctx:setOutputExpression("s",s)
