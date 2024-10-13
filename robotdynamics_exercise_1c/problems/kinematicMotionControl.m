function [ Dq ] = kinematicMotionControl(q, r_des, v_des)
% Inputs:
%  q          : current configuration of the robot
% r_des       : desired end effector position
% v_des       : desired end effector velocity
% Output: joint-space velocity command of the robot.

%User defined linear position gain
K_p = 8;

% Compute the updated joint velocities. This would be used for a velocity controllable robot
r_current = jointToPosition_solution(q);
J_current = jointToPosJac_solution(q);
v_command = v_des + K_p*(r_des - r_current);
Dq = pseudoInverseMat_solution(J_current, 0.01) * v_command;

end
