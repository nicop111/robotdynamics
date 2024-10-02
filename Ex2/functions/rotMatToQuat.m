function q = rotMatToQuat(R)
  % Input: rotation matrix
  % Output: corresponding quaternion [w x y z]
  
  C = R;

  q = (1/2) * [sqrt(C(1,1)+C(2,2)+C(3,3)+1);
                sign(C(3,2)-C(2,3)) * sqrt(C(1,1)-C(2,2)-C(3,3)+1);
                sign(C(1,3)-C(3,1)) * sqrt(-C(1,1)+C(2,2)-C(3,3)+1);
                sign(C(2,1)-C(1,2)) * sqrt(-C(1,1)-C(2,2)+C(3,3)+1)];

end