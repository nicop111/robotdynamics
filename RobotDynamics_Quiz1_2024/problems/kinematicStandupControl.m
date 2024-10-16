function [ Dq ] = kinematicStandupControl( q, p_hip_joint_des, params )  
  % Inputs:
  %  q               : current joint angles (6x1)
  %  p_hip_joint_des : desired hip pose in inertial frame (3x1)
  %  params          : a struct of parameters

  % Output:
  % Dq               : generalized coordinates velocity command (6x1)

  % Proportional controller gain
  K_p = 1;

  % Pseudo_inverse damping coefficient
  lambda = 1e-2;

  % Generalized coordinates
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
  q3 = q(4);
  q4 = q(5);
  q5 = q(6);

  % Jacobians & Inverses
  baseJacobian = jointToBaseJacobian_solution(q, params);
  baseJacobianInverse = pseudoInverseMat_solution(baseJacobian, lambda);
  trayJacobian = jointToTrayJacobian_solution(q,params);
  trayJacobianInverse = pseudoInverseMat_solution(trayJacobian, lambda);

  % Compute recent base pose from q
  T_IB = jointToBasePose(q, params);
  theta = q(3);
  p_base = [T_IB(1:2,4); theta];

  % Compute the desired base pose
  theta_des = p_hip_joint_des(3);
  
  p_base_des = p_hip_joint_des + [[cos(theta_des) -sin(theta_des); sin(theta_des) cos(theta_des)]*[params.l0;params.l1]; 0];

  % Compute the desired base velocities
  w_star_tray = [0; 0; 0];
  w_star_base = p_base_des - p_base;

  Dq = K_p*baseJacobianInverse*w_star_base;

  % Seperatly control q5
  T_IT = jointToTrayPose(q, params);
  angleTray = acos((T_IT(1,1)+T_IT(2,2))/2);
  angleTray_des = 0;
  Dq(6) = 10*K_p * (-angleTray);

end