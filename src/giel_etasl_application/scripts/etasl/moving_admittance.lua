-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Task specification to use admittance
-- KU Leuven 2017
-- ==============================================================================
require("context")
require("geometric")
utils_ts = require("utils_ts")

translation_adm = true
rotation_adm = true
force_tf_tool = false

-- ==================== Input Ports =====================================

Fx_raw = ctx:createInputChannelScalar("Fx")
Fy_raw = ctx:createInputChannelScalar("Fy")
Fz_raw = ctx:createInputChannelScalar("Fz")
Tx_raw = ctx:createInputChannelScalar("Tx")
Ty_raw = ctx:createInputChannelScalar("Ty")
Tz_raw = ctx:createInputChannelScalar("Tz")

W_Fx = ctx:createInputChannelScalar("W_Fx")
W_Fy = ctx:createInputChannelScalar("W_Fy")
W_Fz = ctx:createInputChannelScalar("W_Fz")
W_Tx = ctx:createInputChannelScalar("W_Tx")
W_Ty = ctx:createInputChannelScalar("W_Ty")
W_Tz = ctx:createInputChannelScalar("W_Tz")


-- ========================================= PARAMETERS ===================================

th_f=constant(0.5)
th_t=constant(0.03)

if force_tf_tool then
	tf = task_frame
else
	tf = FT_frame
end

-- ==================================== SIGNALS PRE-PROCESSING ===========================

Fx_lc  = utils_ts.dead_zone(Fx_raw,th_f)
Fy_lc  = utils_ts.dead_zone(Fy_raw,th_f)
Fz_lc  = utils_ts.dead_zone(Fz_raw,th_f)
Tx_lc  = utils_ts.dead_zone(Tx_raw,th_t)
Ty_lc  = utils_ts.dead_zone(Ty_raw,th_t)
Tz_lc  = utils_ts.dead_zone(Tz_raw,th_t)

wr_lc = wrench(vector(Fx_lc,Fy_lc,Fz_lc),vector(Tx_lc,Ty_lc,Tz_lc))

if force_tf_tool then
	wr_tf1  = transform(rotation(T_tf_FT),wr_lc)
	wr_tf   = ref_point(wr_tf1,-origin(T_tf_FT))
else
	wr_tf = wr_lc
end


Fx = coord_x(force(wr_tf))
Fy = coord_y(force(wr_tf))
Fz = coord_z(force(wr_tf))
Tx = coord_x(torque(wr_tf))
Ty = coord_y(torque(wr_tf))
Tz = coord_z(torque(wr_tf))

-- =============================== INITIAL POSE ==============================

t_tf = origin(tf)
r_tf = rotation(tf)

o=initial_value(time,t_tf)		-- initial value of the translation of the end efector
r=initial_value(time,r_tf)		-- initial value of the translation of the end efector

-- =============================== INSTANTANEOUS FRAME ==============================

task_frame_i = inv(make_constant(tf))*tf
pos_vec = origin(task_frame_i)
rot_vec = getRotVec(rotation(task_frame_i))

-- =============================== CONSTRAINT SPECIFICATION ==============================

if translation_adm then
	C_Fx = constant(0.0025) -- compliance in x-axis
	C_Fy = constant(0.0025) -- compliance in y-axis
	C_Fz = constant(0.0025) -- compliance in z-axis
	K_F = constant(4)
	Constraint{
		context=ctx,
		name="follow_force_x",
		model = coord_x(pos_vec),
		meas = -C_Fx*Fx,
		target = 0,
		K = K_F,
		priority = 2,
		weight = abs(W_Fx),
	};

	Constraint{
		context=ctx,
		name="follow_force_y",
		model = coord_y(pos_vec),
		meas = -C_Fy*Fy,
		target = 0,
		K = K_F,
		priority = 2,
		weight = abs(W_Fy),
	};

	Constraint{
		context=ctx,
		name="follow_force_z",
		model = coord_z(pos_vec),
		meas = -C_Fz*Fz,
		target = 0,
		K = K_F,
		priority = 2,
		weight = abs(W_Fz),
	};
else
-- Spring
	Constraint{
		context=ctx,
		name = "Impedance",
		expr = t_tf - o,
		weight = 3,
		priority = 2,
		K = 4
	};
end


if rotation_adm then
	C_Tx = constant(0.09) -- compliance about x-axis
	C_Ty = constant(0.09) -- compliance about y-axis
	C_Tz = constant(0.09) -- compliance about z-axis
	K_T = constant(4)

	Constraint{
		context=ctx,
		name="follow_torque_x",
		model = coord_x(rot_vec),
		meas = -C_Tx*Tx,
		target = 0,
		K = K_T,
		priority = 2,
		weight = abs(W_Tx),
	};

	Constraint{
		context=ctx,
		name="follow_torque_y",
		model = coord_y(rot_vec),
		meas = -C_Ty*Ty,
		target = 0,
		K = K_T,
		priority = 2,
		weight = abs(W_Ty),
	};

	Constraint{
		context=ctx,
		name="follow_torque_z",
		model = coord_z(rot_vec),
		meas = -C_Tz*Tz,
		target = 0,
		K = K_T,
		priority = 2,
		weight = abs(W_Tz),
	};
else
	Constraint {
		context         = ctx,
		name            = "keep_rot",
		expr            = r_tf*inv(r),
		weight          = 1,
		priority        = 2,
		K               = 4
	};
end

-- ============================== OUTPUT THROUGH PORTS===================================

ctx:setOutputExpression("x_tf",coord_x(t_tf))
ctx:setOutputExpression("y_tf",coord_y(t_tf))
ctx:setOutputExpression("z_tf",coord_z(t_tf))
--
roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)
