require("context")
require("geometric")

local u=UrdfExpr();
--u:readFromFile(rospack_find("giel_etasl_application").."/robot_description/urdf/lwr/use_case_setup_lwr.urdf")
u:readFromFile(rospack_find("giel_etasl_application").."/robot_description/urdf/lwr/kuka_b_mode.urdf")
u:addTransform("tool_frame","tip_link","world")

local r = u:getExpressions(ctx)
robot_joints=u:getAllJointNames()

task_frame = r.tool_frame
FT_frame = r.tool_frame
