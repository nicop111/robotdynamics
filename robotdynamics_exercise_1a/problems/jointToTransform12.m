function T12 = jointToTransform12(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  
  phi_deg = (360/2/pi)*q(2);
  
  translation = [0;0;(.29-.145)];
  
  rotation = roty(phi_deg);

  T12 = [rotation translation;
         0 0 0 1];
end