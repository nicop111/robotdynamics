function I_r_IE = jointToRotMat(q)

T0E = jointToT0E(q);
I_r_IE = T0E(1:3,1:3);

end

