require("context")
require("geometric")
require("libexpressiongraph_spline")
utils_ts = require("utils_ts")

-- INFO: https://etasl.pages.gitlab.kuleuven.be/etasl.html

-- ========================================= PARAMETERS ===================================
Fz_raw = ctx:createInputChannelScalar("Fz")

maxvel            = ctx:createInputChannelScalar("maxvel" ,0)
maxacc            = ctx:createInputChannelScalar("maxacc" ,0)
endpose           = ctx:createInputChannelFrame("endpose")
eqradius          = ctx:createInputChannelScalar("eq_r"   ,0)

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
print(startpos)
startrot  = rotation(startpose)

-- =============================== END POSE ==============================
endpos    = origin(endpose)
print(endpos)
endpos[3] = startpos[3]
endrot    = rotation(endpose)
print(endpos)


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
    name = "force_monitor1",
    expr = Fz_raw, -- When expr exceeds the lower or upper bound, the event is triggered (only once).
    upper = -10,
    actionname = "portevent",
    argument = "decrease_force" -- sent flag to state machine
}

Monitor {
    context = ctx,
    name = "force_monitor2",
    expr = Fz_raw, -- When expr exceeds the lower or upper bound, the event is triggered (only once).
    lower = -2,
    actionname = "portevent",
    argument = "increase_force" -- sent flag to state machine
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
