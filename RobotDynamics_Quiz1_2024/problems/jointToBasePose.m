function [ T_IB ] = jointToBasePose(q, params)
  % Inputs:
  %     q: a 6x1 vector of generalized coordinates (6, 1).
  %     params: a struct of parameters.
  %
  % Outputs:
  %     T_IT: Homogenous transform from tray frame {T} to inertial
  %     frame {I}

  % link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;

  % Generalized coordinates
  q0 = q(1);
  q1 = q(2);
  q2 = q(3);
    
  % Implement your solution here ...
  T_I1 = [eye(3), [q0; 0; 0]; 0 0 0 1];

  T_12 = [eye(3), [0; q1; 0]; 0 0 0 1];

  T_2B = [[cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1], [0;0;0]; 0 0 0 1];
  
  T_IB = T_I1 * T_12* T_2B;
  
end

