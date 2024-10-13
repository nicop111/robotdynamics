clear;close;

I_r_IE_des = [0.4;0;0.2];
C_IE_des = rotz(90);
q_0 = zeros(6,1);

q = inverseKinematics(I_r_IE_des, C_IE_des, q_0, 0.001);

disp(q)