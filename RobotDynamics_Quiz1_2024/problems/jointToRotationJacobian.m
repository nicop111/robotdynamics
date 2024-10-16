function [ J_IT_r ] = jointToRotationJacobian(q, params)
  % Inputs:  
  %     q: a 6x1 vector of generalized coordinates
  %     params: a struct of parameters
  %
  % Outputs:
  %     J_IT_r: rotation Jacobian of tray frame {E} wrt inertia frame {I}
  %     expressed in inertial frame {I}
  
  % link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;

  % Generalized coordinates
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
  q3 = q(4);
  q4 = q(5);
  q5 = q(6);
  
  % Implement your solution here...

  % Compute the relative homogeneous transformation matrices.
  T_I1 = [eye(3), [q0; 0; 0]; 0 0 0 1];

  T_12 = [eye(3), [0; q1; 0]; 0 0 0 1];

  T_2B = [[cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1], [0;0;0]; 0 0 0 1];
  
  T_BP1 = [[cos(q3-pi/2) -sin(q3-pi/2) 0; sin(q3-pi/2) cos(q3-pi/2) 0; 0 0 1] [l0; l1; 0]; 0 0 0 1];
 
  T_P1P2 = [[cos(q4) -sin(q4) 0; sin(q4) cos(q4) 0; 0 0 1] [l2; 0; 0]; 0 0 0 1];

  T_P2T = [[cos(q5) -sin(q5) 0; sin(q5) cos(q5) 0; 0 0 1] [l3; 0; 0]; 0 0 0 1];
  
  % Compute the homogeneous transformation matrices from frame k to the inertial frame I.
  T_I1 = T_I1;
  T_I2 = T_I1*T_12;
  T_IB = T_I2*T_2B;
  T_IP1 = T_IB*T_BP1;
  T_IP2 = T_IP1*T_P1P2;
  T_IT = T_IP2*T_P2T;

  
  % Extract the rotation matrices from each homogeneous transformation matrix.
  R_I1 = T_I1(1:3,1:3);
  R_I2 = T_I2(1:3,1:3);
  R_IB = T_IB(1:3,1:3);
  R_IP1 = T_IP1(1:3,1:3);
  R_IP2 = T_IP2(1:3,1:3);
  R_IT = T_IT(1:3,1:3);
  
  % Define the unit vectors around which each link rotates in the precedent coordinate frame.
  n_1 = [0;0;1];
  n_2 = [0;0;1];
  n_3 = [0;0;1];
  n_4 = [0;0;1];
  n_5 = [0;0;1];
  n_6 = [0;0;1];
  
  % Compute the rotational jacobian.
  J_IT_r = [   R_I1*n_1 ...
            R_I2*n_2 ...
            R_IB*n_3 ...
            R_IP1*n_4 ...
            R_IP2*n_5 ...
            R_IT*n_6 ...
        ];
  
end

