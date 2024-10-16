function [ phi ] = rotMatToRotVec(C)
% Input: a rotation matrix C
% Output: the rotational vector which describes the rotation C

theta = acos((C(1,1)+C(2,2)+C(3,3)-1)/2);

phi = theta/2/sin(theta) * [C(3,2)-C(2,3); C(1,3)-C(3,1); C(2,1)-C(1,2)];

end
