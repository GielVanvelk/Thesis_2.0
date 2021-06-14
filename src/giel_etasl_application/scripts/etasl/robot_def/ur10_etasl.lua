require("context")
require("geometric")

-- loading a model for the unversal robot UR10:
local u=UrdfExpr();
u:readFromFile(rospack_find("giel_etasl_application").."/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
u:addTransform("ee","tool0","base")
u:addTransform("FT_frame","load_cell_link","base")
u:addTransform("TCP_frame","TCP_frame","base")
u:addTransform("T_TCP_FT","load_cell_link","TCP_frame")			-- Transformation from the hook to the load cell

local r = u:getExpressions(ctx)

robot_ee = r.ee
FT_frame = r.FT_frame
task_frame = r.TCP_frame
T_tf_FT = r.T_TCP_FT

robot_joints={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}
