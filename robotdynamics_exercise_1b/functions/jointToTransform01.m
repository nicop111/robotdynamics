function T01 = jointToTransform01(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  
  phi_deg = (360/2/pi)*q(1);
  
  translation = [0;0;0.145];
  
  rotation = rotz(phi_deg);

  T01 = [rotation translation;
         0 0 0 1];
end