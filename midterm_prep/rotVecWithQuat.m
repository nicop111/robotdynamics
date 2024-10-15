function A_r = rotVecWithQuat(quat_AB,B_r)

C_AB = quatToRotMat(quat_AB);
A_r = C_AB*B_r;

end
