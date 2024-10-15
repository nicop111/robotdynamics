function I_r_IE = jointToPosition(q)

T0E = jointToT0E(q);
I_r_IE = T0E(1:3,4);

end

