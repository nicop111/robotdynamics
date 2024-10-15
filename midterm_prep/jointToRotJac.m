function J_R = jointToRotJac(q)
    
  % Compute the relative homogeneous transformation matrices.
  T_I0 = jointToTransformI0(q);
  T_01 = jointToTransform01(q);
  T_12 = jointToTransform12(q);
  T_23 = jointToTransform23(q);
  T_34 = jointToTransform34(q);
  T_45 = jointToTransform45(q);
  T_56 = %jointToTransform56(q);
  
  % Compute the homogeneous transformation matrices from frame k to the inertial frame I.
  T_I1 = T_I0*T_01;
  T_I2 = T_I1*T_12;
  T_I3 = T_I2*T_23;
  T_I4 = T_I3*T_34;
  T_I5 = T_I4*T_45;
  T_I6 = T_I5*T_56;
  
  % Extract the rotation matrices from each homogeneous transformation matrix.
  R_I1 = T_I1(1:3,1:3);
  R_I2 = T_I2(1:3,1:3);
  R_I3 = T_I3(1:3,1:3);
  R_I4 = T_I4(1:3,1:3);
  R_I5 = T_I5(1:3,1:3);
  R_I6 = T_I6(1:3,1:3);
  
  % Define the unit vectors around which each link rotates in the precedent coordinate frame.
  n_1 = %[1;0;0];
  n_2 = [1;0;0];
  n_3 = [1;0;0];
  n_4 = [1;0;0];
  n_5 = [1;0;0];
  n_6 = [1;0;0];
  
  % Compute the rotational jacobian.
  J_R = %[   R_I1*n_1 ...
            R_I2*n_2 ...
            R_I3*n_3 ...
            R_I4*n_4 ...
            R_I5*n_5 ...
            R_I6*n_6 ...
        ];
  
end