function [ Dq ] = kinematicTrajectoryControl( q, p_base_des, params )  
  % Inputs:
  %  q             : current joint angles (6x1)
  %  p_base_des    : desired 2D base pose (x,y,q1) (3x1)
  %  params        : a struct of parameters

  % Output:
  % Dq             : generalized coordinates velocity command (6x1)

  % Proportional controller gain
  K_p = 10;

  % Pseudo_inverse damping coefficient
  lambda = 1e-2;

  % Generalized coordinates
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
  q3 = q(4);
  q4 = q(5);
  q5 = q(6);

  % Implement your solution here...
  baseJacobian = jointToBaseJacobian_solution(q, params);
  baseJacobianInverse = pseudoInverseMat_solution(baseJacobian, lambda);
  trayJacobian = jointToTrayJacobian_solution(q,params);
  trayJacobianInverse = pseudoInverseMat_solution(trayJacobian, lambda);
    
  trayNull = eye(6) - trayJacobianInverse*trayJacobian; %Tray nullspace

  % Compute recent base pose from q
  T_IB = jointToBasePose(q, params);
  thata = q(3);
  p_base = [T_IB(1:2,4); thata];

  w_star_tray = [0; 0; 0];
  w_star_base = p_base_des - p_base;

  %Tray must be prioritized --> project the desired base pose to the tray nullspace
  Dq = trayJacobianInverse * w_star_tray + trayNull*pseudoInverseMat_solution(baseJacobian*trayNull, lambda)*(w_star_base - baseJacobian*trayJacobianInverse*w_star_tray); 
  Dq = K_p*Dq;
end