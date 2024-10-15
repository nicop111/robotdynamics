function Dq = kinematicMotionControl(q, r_des, v_des)

K_p = 5; % Position gain
lambda = 0.1; % Pseudo-inverse damping

r_current = jointToPosition(q);
J_current = jointToPosJac(q);
v_command = v_des + K_p*(r_des - r_current);
Dq = pseudoInverseMat_solution(J_current, lambda) * v_command;

end
