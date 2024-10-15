function C = quatToRotMat(quat)
  
  quat_w = quat(1); % Scalar part of the quaternion.
  quat_n = quat(2:4); % Vector part of the quaternion.
  
  C = (2*quat_w^2-1)*eye(3) + 2.0*quat_w*skewMatrix(quat_n) + 2.0*(quat_n*quat_n');
end

function A = skewMatrix(q_n)
    A = [0, -q_n(3), q_n(2);...
         q_n(3), 0, -q_n(1);...
        -q_n(2), q_n(1), 0];
end
