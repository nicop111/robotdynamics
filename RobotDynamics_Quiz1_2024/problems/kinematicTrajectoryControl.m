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
    
  baseN = eye(6) - baseJacobianInverse*baseJacobian;
  trayN = eye(6) - trayJacobianInverse*trayJacobian;

  %tray must be prioritized
  Dq = trayN*pseudoInverseMat_solution(baseJacobian*trayN, lambda)*p_base_des; 
end