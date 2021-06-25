require("context")
require("geometric")
require("libexpressiongraph_spline")
utils_ts = require("utils_ts")

-- INFO: https://etasl.pages.gitlab.kuleuven.be/etasl.html

-- ========================================= PARAMETERS ===================================
-- Default value is optional

maxvel    = ctx:createInputChannelScalar("maxvel" ,0)
maxacc    = ctx:createInputChannelScalar("maxacc" ,0)

eqradius  = ctx:createInputChannelScalar("eq_r"   ,0)

delta_x   = ctx:createInputChannelScalar("delta_x",0)
delta_y   = ctx:createInputChannelScalar("delta_y",0)
delta_z   = ctx:createInputChannelScalar("delta_z",0)

force     = ctx:createInputChannelScalar("force",0)


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

-- ========================== CONSTRAINT SPECIFICATION =================================
Constraint{
    context = ctx,
    name    = "follow_path",
    expr    = inv(target)*tf,
    K       = 3,
    weight  = 1,
    priority= 2
}

--[[
Constraint{
    context = ctx,
    name    = "upper_force_limit",
    expr    = inv(target)*tf,
    K       = 3,
    weight  = 1,
    priority= 3
}

Constraint{
    context = ctx,
    name    = "lower_force_limit",
    expr    = inv(target)*tf,
    K       = 3,
    weight  = 1,
    priority= 3
}


Constraint{
   context = ...
   name = ...         [optional, default_name<nr>]
   model = ...  (expression)
   meas = ...   (expression)
   expr = ...[ compatibility, if expr is used then model==expr and meas==expr ]
   target = ...       [optional, 0.0] ( can be expression )
   target_lower = ... [optional, 0.0] ( can be expression )
   target_upper = ... [optional, 0.0] ( can be expression )
   weight = ...       [optional, defaultweight, >= 0] ( can be expression )
   priority = ...     [optional, defaultpriority, 0..2]
   controller_lower = ....  [optional, 'proportional']
   controller_upper = ....  [optional, 'proportional']
   controller       = ....  [optional, 'proportional']
   <controllerparameter> =... (can be expressions)
   <controllerparameter>_lower or <controllerparameter>_upper (can be expressions)
}
--]]

-- =========================== MONITOR ============================================
Monitor{
        context=ctx,
        name='finish_after_motion',
        upper=0.0,
        actionname='exit',
        expr=time-get_duration(mp) - constant(0.1)
}

--[[
Monitor {
    context = ctx,
    name = "force_monitor"
    expr   = force, -- When expr exceeds the lower or upper bound, the event is triggered (only once).
    lower = 3.00,
    upper = 6.00,
    actionname = "portevent",
    argument = "e_force_too_low" -- sent flag to state machine
}

--]]
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
